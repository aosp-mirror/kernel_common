// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 Google LLC
 */
#define _GNU_SOURCE

#include "test_fuse.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mount.h>
#include <sys/wait.h>

#include <linux/random.h>

#include <include/uapi/linux/fuse.h>
#include <include/uapi/linux/bpf.h>

static const char *ft_src = "ft-src";
static const char *ft_dst = "ft-dst";

static void fill_buffer(uint8_t *data, size_t len, int file, int block)
{
	int i;
	int seed = 7919 * file + block;

	for (i = 0; i < len; i++) {
		seed = 1103515245 * seed + 12345;
		data[i] = (uint8_t)(seed >> (i % 13));
	}
}

static bool test_buffer(uint8_t *data, size_t len, int file, int block)
{
	int i;
	int seed = 7919 * file + block;

	for (i = 0; i < len; i++) {
		seed = 1103515245 * seed + 12345;
		if (data[i] != (uint8_t)(seed >> (i % 13)))
			return false;
	}

	return true;
}

static int create_file(int dir, struct s name, int index, size_t blocks)
{
	int result = TEST_FAILURE;
	int fd = -1;
	int i;
	uint8_t data[PAGE_SIZE];

	TEST(fd = s_openat(dir, name, O_CREAT | O_WRONLY, 0777), fd != -1);
	for (i = 0; i < blocks; ++i) {
		fill_buffer(data, PAGE_SIZE, index, i);
		TESTEQUAL(write(fd, data, sizeof(data)), PAGE_SIZE);
	}
	TESTSYSCALL(close(fd));
	result = TEST_SUCCESS;

out:
	close(fd);
	return result;
}

static int bpf_clear_trace(void)
{
	int result = TEST_FAILURE;
	int tp = -1;

	TEST(tp = s_open(s_path(tracing_folder(), s("trace")),
			 O_WRONLY | O_TRUNC | O_CLOEXEC), tp != -1);

	result = TEST_SUCCESS;
out:
	close(tp);
	return result;
}

static int bpf_test_trace_maybe(const char *substr, bool present)
{
	int result = TEST_FAILURE;
	int tp = -1;
	char trace_buffer[4096] = {};
	ssize_t bytes_read;

	TEST(tp = s_open(s_path(tracing_folder(), s("trace_pipe")),
			 O_RDONLY | O_CLOEXEC),
	     tp != -1);
	fcntl(tp, F_SETFL, O_NONBLOCK);

	for (;;) {
		bytes_read = read(tp, trace_buffer, sizeof(trace_buffer));
		if (present)
			TESTCOND(bytes_read > 0);
		else if (bytes_read <= 0) {
			result = TEST_SUCCESS;
			break;
		}

		if (test_options.verbose)
			ksft_print_msg("%s\n", trace_buffer);

		if (strstr(trace_buffer, substr)) {
			if (present)
				result = TEST_SUCCESS;
			break;
		}
	}
out:
	close(tp);
	return result;
}

static int bpf_test_trace(const char *substr)
{
	return bpf_test_trace_maybe(substr, true);
}

static int bpf_test_no_trace(const char *substr)
{
	return bpf_test_trace_maybe(substr, false);
}

static int basic_test(const char *mount_dir)
{
	const char *test_name = "test";
	const char *test_data = "data";

	int result = TEST_FAILURE;
	int fuse_dev = -1;
	char *filename = NULL;
	int fd = -1;
	int pid = -1;
	int status;

	TESTEQUAL(mount_fuse(mount_dir, -1, -1, &fuse_dev), 0);
	FUSE_ACTION
		char data[256];

		filename = concat_file_name(mount_dir, test_name);
		TESTERR(fd = open(filename, O_RDONLY | O_CLOEXEC), fd != -1);
		TESTEQUAL(read(fd, data, strlen(test_data)), strlen(test_data));
		TESTCOND(!strcmp(data, test_data));
		TESTSYSCALL(close(fd));
		fd = -1;
	FUSE_DAEMON
		DECL_FUSE_IN(open);
		DECL_FUSE_IN(read);
		DECL_FUSE_IN(flush);
		DECL_FUSE_IN(release);

		TESTFUSELOOKUP(test_name, 0);
		TESTFUSEOUT1(fuse_entry_out, ((struct fuse_entry_out) {
			.nodeid		= 2,
			.generation	= 1,
			.attr.ino = 100,
			.attr.size = 4,
			.attr.blksize = 512,
			.attr.mode = S_IFREG | 0777,
			}));

		TESTFUSEIN(FUSE_OPEN, open_in);
		TESTFUSEOUT1(fuse_open_out, ((struct fuse_open_out) {
			.fh = 1,
			.open_flags = open_in->flags,
		}));

		TESTFUSEIN(FUSE_READ, read_in);
		TESTFUSEOUTREAD(test_data, strlen(test_data));

		TESTFUSEIN(FUSE_FLUSH, flush_in);
		TESTFUSEOUTEMPTY();

		TESTFUSEIN(FUSE_RELEASE, release_in);
		TESTFUSEOUTEMPTY();
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	if (!pid)
		exit(TEST_FAILURE);
	close(fuse_dev);
	close(fd);
	free(filename);
	umount(mount_dir);
	return result;
}

static int bpf_test_real(const char *mount_dir)
{
	const char *test_name = "real";
	const char *test_data = "Weebles wobble but they don't fall down";
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	char *filename = NULL;
	int fd = -1;
	char read_buffer[256] = {};
	ssize_t bytes_read;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TEST(fd = openat(src_fd, test_name, O_CREAT | O_RDWR | O_CLOEXEC, 0777),
	     fd != -1);
	TESTEQUAL(write(fd, test_data, strlen(test_data)), strlen(test_data));
	TESTSYSCALL(close(fd));
	fd = -1;

	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	filename = concat_file_name(mount_dir, test_name);
	TESTERR(fd = open(filename, O_RDONLY | O_CLOEXEC), fd != -1);
	bytes_read = read(fd, read_buffer, strlen(test_data));
	TESTEQUAL(bytes_read, strlen(test_data));
	TESTEQUAL(strcmp(test_data, read_buffer), 0);
	TESTEQUAL(bpf_test_trace("read"), 0);

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	close(fd);
	free(filename);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}


static int bpf_test_partial(const char *mount_dir)
{
	const char *test_name = "partial";
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	char *filename = NULL;
	int fd = -1;
	int pid = -1;
	int status;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(create_file(src_fd, s(test_name), 1, 2), 0);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		uint8_t data[PAGE_SIZE];

		TEST(filename = concat_file_name(mount_dir, test_name),
		     filename);
		TESTERR(fd = open(filename, O_RDONLY | O_CLOEXEC), fd != -1);
		TESTEQUAL(read(fd, data, PAGE_SIZE), PAGE_SIZE);
		TESTEQUAL(bpf_test_trace("read"), 0);
		TESTCOND(test_buffer(data, PAGE_SIZE, 2, 0));
		TESTCOND(!test_buffer(data, PAGE_SIZE, 1, 0));
		TESTEQUAL(read(fd, data, PAGE_SIZE), PAGE_SIZE);
		TESTCOND(test_buffer(data, PAGE_SIZE, 1, 1));
		TESTCOND(!test_buffer(data, PAGE_SIZE, 2, 1));
		TESTSYSCALL(close(fd));
		fd = -1;
	FUSE_DAEMON
		DECL_FUSE(open);
		DECL_FUSE(read);
		DECL_FUSE(release);
		uint8_t data[PAGE_SIZE];

		TESTFUSEIN2(FUSE_OPEN | FUSE_POSTFILTER, open_in, open_out);
		TESTFUSEOUT1(fuse_open_out, ((struct fuse_open_out) {
			.fh = 1,
			.open_flags = open_in->flags,
		}));

		TESTFUSEIN(FUSE_READ, read_in);
		fill_buffer(data, PAGE_SIZE, 2, 0);
		TESTFUSEOUTREAD(data, PAGE_SIZE);

		TESTFUSEIN(FUSE_RELEASE, release_in);
		TESTFUSEOUTEMPTY();
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	if (!pid)
		exit(TEST_FAILURE);
	close(fuse_dev);
	close(fd);
	free(filename);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}

static int bpf_test_attrs(const char *mount_dir)
{
	const char *test_name = "partial";
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	char *filename = NULL;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(create_file(src_fd, s(test_name), 1, 2), 0);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TEST(filename = concat_file_name(mount_dir, test_name), filename);
	TESTSYSCALL(stat(filename, &st));
	TESTSYSCALL(chmod(filename, 0111));
	TESTSYSCALL(stat(filename, &st));
	TESTEQUAL(st.st_mode & 0777, 0111);
	TESTSYSCALL(chmod(filename, 0777));
	TESTSYSCALL(stat(filename, &st));
	TESTEQUAL(st.st_mode & 0777, 0777);
	TESTSYSCALL(chown(filename, 5, 6));
	TESTSYSCALL(stat(filename, &st));
	TESTEQUAL(st.st_uid, 5);
	TESTEQUAL(st.st_gid, 6);

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	free(filename);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}

static int bpf_test_readdir(const char *mount_dir)
{
	const char *names[] = {"real", "partial", "fake", ".", ".."};
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	int pid = -1;
	int status;
	DIR *dir = NULL;
	struct dirent *dirent;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(create_file(src_fd, s(names[0]), 1, 2), 0);
	TESTEQUAL(create_file(src_fd, s(names[1]), 1, 2), 0);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		int i, j;

		TEST(dir = s_opendir(s(mount_dir)), dir);
		TESTEQUAL(bpf_test_trace("opendir"), 0);

		for (i = 0; i < ARRAY_SIZE(names); ++i) {
			TEST(dirent = readdir(dir), dirent);

			for (j = 0; j < ARRAY_SIZE(names); ++j)
				if (names[j] &&
				    strcmp(names[j], dirent->d_name) == 0) {
					names[j] = NULL;
					break;
				}
			TESTNE(j, ARRAY_SIZE(names));
		}
		TEST(dirent = readdir(dir), dirent == NULL);
		TESTSYSCALL(closedir(dir));
		dir = NULL;
		TESTEQUAL(bpf_test_trace("readdir"), 0);
	FUSE_DAEMON
		struct fuse_in_header *in_header =
			(struct fuse_in_header *)bytes_in;
		ssize_t res = read(fuse_dev, bytes_in, sizeof(bytes_in));
		struct fuse_read_out *read_out =
			(struct fuse_read_out *) (bytes_in +
					sizeof(*in_header) +
					sizeof(struct fuse_read_in));
		struct fuse_dirent *fuse_dirent =
			(struct fuse_dirent *) (bytes_in + res);

		TESTGE(res, sizeof(*in_header) + sizeof(struct fuse_read_in));
		TESTEQUAL(in_header->opcode, FUSE_READDIR | FUSE_POSTFILTER);
		*fuse_dirent = (struct fuse_dirent) {
			.ino = 100,
			.off = 5,
			.namelen = strlen("fake"),
			.type = DT_REG,
		};
		strcpy((char *)(bytes_in + res + sizeof(*fuse_dirent)), "fake");
		res += FUSE_DIRENT_ALIGN(sizeof(*fuse_dirent) + strlen("fake") +
					 1);
		TESTFUSEDIROUTREAD(read_out,
				bytes_in +
				   sizeof(struct fuse_in_header) +
				   sizeof(struct fuse_read_in) +
				   sizeof(struct fuse_read_out),
				res - sizeof(struct fuse_in_header) -
				    sizeof(struct fuse_read_in) -
				    sizeof(struct fuse_read_out));
		res = read(fuse_dev, bytes_in, sizeof(bytes_in));
		TESTEQUAL(res, sizeof(*in_header) +
			  sizeof(struct fuse_read_in) +
			  sizeof(struct fuse_read_out));
		TESTEQUAL(in_header->opcode, FUSE_READDIR | FUSE_POSTFILTER);
		TESTFUSEDIROUTREAD(read_out, bytes_in, 0);
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	closedir(dir);
	close(fuse_dev);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}

static int bpf_test_redact_readdir(const char *mount_dir)
{
	const char *names[] = {"f1", "f2", "f3", "f4", "f5", "f6", ".", ".."};
	int num_shown = (ARRAY_SIZE(names) - 2) / 2 + 2;
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	int pid = -1;
	int status;
	DIR *dir = NULL;
	struct dirent *dirent;
	int i;
	int count = 0;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	for (i = 0; i < ARRAY_SIZE(names) - 2; i++)
		TESTEQUAL(create_file(src_fd, s(names[i]), 1, 2), 0);

	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_readdir_redact",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		int j;

		TEST(dir = s_opendir(s(mount_dir)), dir);
		while ((dirent = readdir(dir))) {
			errno = 0;
			TESTEQUAL(errno, 0);

			for (j = 0; j < ARRAY_SIZE(names); ++j)
				if (names[j] &&
				    strcmp(names[j], dirent->d_name) == 0) {
					names[j] = NULL;
					count++;
					break;
				}
			TESTNE(j, ARRAY_SIZE(names));
			TESTGE(num_shown, count);
		}
		TESTEQUAL(count, num_shown);
		TESTSYSCALL(closedir(dir));
		dir = NULL;
	FUSE_DAEMON
	bool skip = true;
		for (int i = 0; i < ARRAY_SIZE(names) + 1; i++) {
			uint8_t bytes_in[FUSE_MIN_READ_BUFFER];
			uint8_t bytes_out[FUSE_MIN_READ_BUFFER];
			struct fuse_in_header *in_header =
				(struct fuse_in_header *)bytes_in;
			ssize_t res = read(fuse_dev, bytes_in, sizeof(bytes_in));
			int length_out = 0;
			uint8_t *pos;
			uint8_t *dirs_in;
			uint8_t *dirs_out;
			struct fuse_read_in *fuse_read_in;
			struct fuse_read_out *fuse_read_out_in;
			struct fuse_read_out *fuse_read_out_out;
			struct fuse_dirent *fuse_dirent_in = NULL;
			struct fuse_dirent *next = NULL;
			bool again = false;
			int dir_ent_len = 0;

			TESTGE(res, sizeof(struct fuse_in_header) +
					sizeof(struct fuse_read_in) +
					sizeof(struct fuse_read_out));

			pos = bytes_in + sizeof(struct fuse_in_header);
			fuse_read_in = (struct fuse_read_in *) pos;
			pos += sizeof(*fuse_read_in);
			fuse_read_out_in = (struct fuse_read_out *) pos;
			pos += sizeof(*fuse_read_out_in);
			dirs_in = pos;

			pos = bytes_out + sizeof(struct fuse_out_header);
			fuse_read_out_out = (struct fuse_read_out *) pos;
			pos += sizeof(*fuse_read_out_out);
			dirs_out = pos;

			if (dirs_in < bytes_in + res) {
				bool is_dot;

				fuse_dirent_in = (struct fuse_dirent *) dirs_in;
				is_dot = !strcmp(fuse_dirent_in->name, ".") ||
					 !strcmp(fuse_dirent_in->name, "..");

				dir_ent_len = FUSE_DIRENT_ALIGN(
					sizeof(*fuse_dirent_in) +
					fuse_dirent_in->namelen + 1);

				if (dirs_in + dir_ent_len < bytes_in + res)
					next = (struct fuse_dirent *)
							(dirs_in + dir_ent_len);

				if (!skip || is_dot) {
					memcpy(dirs_out, fuse_dirent_in,
					       sizeof(struct fuse_dirent) +
					       fuse_dirent_in->namelen + 1);
					length_out += dir_ent_len;
				}
				again = ((skip && !is_dot) && next);

				if (!is_dot)
					skip = !skip;
			}

			fuse_read_out_out->offset = next ? next->off :
					fuse_read_out_in->offset;
			fuse_read_out_out->again = again;

			{
			struct fuse_out_header *out_header =
				(struct fuse_out_header *)bytes_out;

			*out_header = (struct fuse_out_header) {
				.len = sizeof(*out_header) +
				       sizeof(*fuse_read_out_out) + length_out,
				.unique = in_header->unique,
			};
			TESTEQUAL(write(fuse_dev, bytes_out, out_header->len),
				  out_header->len);
			}
		}
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	closedir(dir);
	close(fuse_dev);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}

/*
 * This test is more to show what classic fuse does with a creat in a subdir
 * than a test of any new functionality
 */
static int bpf_test_creat(const char *mount_dir)
{
	const char *dir_name = "show";
	const char *file_name = "file";
	int result = TEST_FAILURE;
	int fuse_dev = -1;
	int pid = -1;
	int status;
	int fd = -1;

	TESTEQUAL(mount_fuse(mount_dir, -1, -1, &fuse_dev), 0);

	FUSE_ACTION
		TEST(fd = s_creat(s_path(s_path(s(mount_dir), s(dir_name)),
					 s(file_name)),
				  0777),
		     fd != -1);
		TESTSYSCALL(close(fd));
	FUSE_DAEMON
		DECL_FUSE_IN(create);
		DECL_FUSE_IN(release);
		DECL_FUSE_IN(flush);

		TESTFUSELOOKUP(dir_name, 0);
		TESTFUSEOUT1(fuse_entry_out, ((struct fuse_entry_out) {
			.nodeid		= 3,
			.generation	= 1,
			.attr.ino = 100,
			.attr.size = 4,
			.attr.blksize = 512,
			.attr.mode = S_IFDIR | 0777,
			}));

		TESTFUSELOOKUP(file_name, 0);
		TESTFUSEOUTERROR(-ENOENT);

		TESTFUSEINEXT(FUSE_CREATE, create_in, strlen(file_name) + 1);
		TESTFUSEOUT2(fuse_entry_out, ((struct fuse_entry_out) {
			.nodeid		= 2,
			.generation	= 1,
			.attr.ino = 200,
			.attr.size = 4,
			.attr.blksize = 512,
			.attr.mode = S_IFREG,
			}),
			fuse_open_out, ((struct fuse_open_out) {
			.fh = 1,
			.open_flags = create_in->flags,
			}));

		TESTFUSEIN(FUSE_FLUSH, flush_in);
		TESTFUSEOUTEMPTY();

		TESTFUSEIN(FUSE_RELEASE, release_in);
		TESTFUSEOUTEMPTY();
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	return result;
}

static int bpf_test_hidden_entries(const char *mount_dir)
{
	static const char * const dir_names[] = {
		"show",
		"hide",
	};
	const char *file_name = "file";
	const char *data = "The quick brown fox jumps over the lazy dog\n";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	int fd = -1;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTSYSCALL(mkdirat(src_fd, dir_names[0], 0777));
	TESTSYSCALL(mkdirat(src_fd, dir_names[1], 0777));
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_hidden",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TEST(fd = s_creat(s_path(s_path(s(mount_dir), s(dir_names[0])),
				 s(file_name)),
			  0777),
	     fd != -1);
	TESTSYSCALL(fallocate(fd, 0, 0, 4096));
	TEST(write(fd, data, strlen(data)), strlen(data));
	TESTSYSCALL(close(fd));
	TESTEQUAL(bpf_test_trace("Create"), 0);

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_dir(const char *mount_dir)
{
	const char *dir_name = "dir";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_mkdir(s_path(s(mount_dir), s(dir_name)), 0777));
	TESTEQUAL(bpf_test_trace("mkdir"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(dir_name)), &st));
	TESTSYSCALL(s_rmdir(s_path(s(mount_dir), s(dir_name))));
	TESTEQUAL(s_stat(s_path(s(ft_src), s(dir_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);
	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_file(const char *mount_dir, bool close_first)
{
	const char *file_name = "real";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	int fd = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
			  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TEST(fd = s_creat(s_path(s(mount_dir), s(file_name)),
			  0777),
	     fd != -1);
	TESTEQUAL(bpf_test_trace("Create"), 0);
	if (close_first) {
		TESTSYSCALL(close(fd));
		fd = -1;
	}
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(file_name)), &st));
	TESTSYSCALL(s_unlink(s_path(s(mount_dir), s(file_name))));
	TESTEQUAL(bpf_test_trace("unlink"), 0);
	TESTEQUAL(s_stat(s_path(s(ft_src), s(file_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);
	if (!close_first) {
		TESTSYSCALL(close(fd));
		fd = -1;
	}
	result = TEST_SUCCESS;
out:
	close(fd);
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_file_early_close(const char *mount_dir)
{
	return bpf_test_file(mount_dir, true);
}

static int bpf_test_file_late_close(const char *mount_dir)
{
	return bpf_test_file(mount_dir, false);
}

static int bpf_test_alter_errcode_bpf(const char *mount_dir)
{
	const char *dir_name = "dir";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_error",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_mkdir(s_path(s(mount_dir), s(dir_name)), 0777));
	//TESTEQUAL(bpf_test_trace("mkdir"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(dir_name)), &st));
	TESTEQUAL(s_mkdir(s_path(s(mount_dir), s(dir_name)), 0777), -EPERM);
	TESTSYSCALL(s_rmdir(s_path(s(mount_dir), s(dir_name))));
	TESTEQUAL(s_stat(s_path(s(ft_src), s(dir_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);
	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_alter_errcode_userspace(const char *mount_dir)
{
	const char *dir_name = "doesnotexist";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	int pid = -1;
	int status;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_error",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		TESTEQUAL(s_unlink(s_path(s(mount_dir), s(dir_name))),
		     -1);
		TESTEQUAL(errno, ENOMEM);
	FUSE_DAEMON
		TESTFUSELOOKUP("doesnotexist", FUSE_POSTFILTER);
		TESTFUSEOUTERROR(-ENOMEM);
	FUSE_DONE
	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_mknod(const char *mount_dir)
{
	const char *file_name = "real";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_mkfifo(s_path(s(mount_dir), s(file_name)), 0777));
	TESTEQUAL(bpf_test_trace("mknod"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(file_name)), &st));
	TESTSYSCALL(s_unlink(s_path(s(mount_dir), s(file_name))));
	TESTEQUAL(bpf_test_trace("unlink"), 0);
	TESTEQUAL(s_stat(s_path(s(ft_src), s(file_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);
	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_largedir(const char *mount_dir)
{
	const char *show = "show";
	const int files = 1000;

	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct map_relocation *map_relocations = NULL;
	size_t map_count = 0;
	int pid = -1;
	int status;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("fd_bpf.bpf", "test_daemon",
			  &bpf_fd, &map_relocations, &map_count), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		int i;
		int fd;
		DIR *dir = NULL;
		struct dirent *dirent;

		TESTSYSCALL(s_mkdir(s_path(s(mount_dir), s(show)), 0777));
		for (i = 0; i < files; ++i) {
			char filename[NAME_MAX];

			sprintf(filename, "%d", i);
			TEST(fd = s_creat(s_path(s_path(s(mount_dir), s(show)),
						 s(filename)), 0777), fd != -1);
			TESTSYSCALL(close(fd));
		}

		TEST(dir = s_opendir(s_path(s(mount_dir), s(show))), dir);
		for (dirent = readdir(dir); dirent; dirent = readdir(dir))
			;
		closedir(dir);
	FUSE_DAEMON
		int i;

		for (i = 0; i < files + 2; ++i) {
			TESTFUSELOOKUP(show, FUSE_PREFILTER);
			TESTFUSEOUTREAD(show, 5);
		}
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_link(const char *mount_dir)
{
	const char *file_name = "real";
	const char *link_name = "partial";
	int result = TEST_FAILURE;
	int fd = -1;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace", &bpf_fd, NULL,
				  NULL),
		  0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TEST(fd = s_creat(s_path(s(mount_dir), s(file_name)), 0777), fd != -1);
	TESTEQUAL(bpf_test_trace("Create"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(file_name)), &st));

	TESTSYSCALL(s_link(s_path(s(mount_dir), s(file_name)),
			   s_path(s(mount_dir), s(link_name))));

	TESTEQUAL(bpf_test_trace("link"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(link_name)), &st));

	TESTSYSCALL(s_unlink(s_path(s(mount_dir), s(link_name))));
	TESTEQUAL(bpf_test_trace("unlink"), 0);
	TESTEQUAL(s_stat(s_path(s(ft_src), s(link_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);

	TESTSYSCALL(s_unlink(s_path(s(mount_dir), s(file_name))));
	TESTEQUAL(bpf_test_trace("unlink"), 0);
	TESTEQUAL(s_stat(s_path(s(ft_src), s(file_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);

	result = TEST_SUCCESS;
out:
	close(fd);
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_symlink(const char *mount_dir)
{
	const char *test_name = "real";
	const char *symlink_name = "partial";
	const char *test_data = "Weebles wobble but they don't fall down";
	int result = TEST_FAILURE;
	int bpf_fd = -1;
	int src_fd = -1;
	int fuse_dev = -1;
	int fd = -1;
	char read_buffer[256] = {};
	ssize_t bytes_read;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TEST(fd = openat(src_fd, test_name, O_CREAT | O_RDWR | O_CLOEXEC, 0777),
	     fd != -1);
	TESTEQUAL(write(fd, test_data, strlen(test_data)), strlen(test_data));
	TESTSYSCALL(close(fd));
	fd = -1;

	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_symlink(s_path(s(mount_dir), s(test_name)),
				   s_path(s(mount_dir), s(symlink_name))));
	TESTEQUAL(bpf_test_trace("symlink"), 0);

	TESTERR(fd = s_open(s_path(s(mount_dir), s(symlink_name)), O_RDONLY | O_CLOEXEC), fd != -1);
	bytes_read = read(fd, read_buffer, strlen(test_data));
	TESTEQUAL(bpf_test_trace("readlink"), 0);
	TESTEQUAL(bytes_read, strlen(test_data));
	TESTEQUAL(strcmp(test_data, read_buffer), 0);

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	close(fd);
	umount(mount_dir);
	close(src_fd);
	close(bpf_fd);
	return result;
}

static int bpf_test_xattr(const char *mount_dir)
{
	static const char file_name[] = "real";
	static const char xattr_name[] = "user.xattr_test_name";
	static const char xattr_value[] = "this_is_a_test";
	const size_t xattr_size = sizeof(xattr_value);
	char xattr_value_ret[256];
	ssize_t xattr_size_ret;
	int result = TEST_FAILURE;
	int fd = -1;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	memset(xattr_value_ret, '\0', sizeof(xattr_value_ret));

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace", &bpf_fd, NULL,
				  NULL),
		  0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TEST(fd = s_creat(s_path(s(mount_dir), s(file_name)), 0777), fd != -1);
	TESTEQUAL(bpf_test_trace("Create"), 0);
	TESTSYSCALL(close(fd));

	TESTSYSCALL(s_stat(s_path(s(ft_src), s(file_name)), &st));
	TEST(result = s_getxattr(s_path(s(mount_dir), s(file_name)), xattr_name,
				 xattr_value_ret, sizeof(xattr_value_ret),
				 &xattr_size_ret),
	     result == -1);
	TESTEQUAL(errno, ENODATA);
	TESTEQUAL(bpf_test_trace("getxattr"), 0);

	TESTSYSCALL(s_listxattr(s_path(s(mount_dir), s(file_name)),
				xattr_value_ret, sizeof(xattr_value_ret),
				&xattr_size_ret));
	TESTEQUAL(bpf_test_trace("listxattr"), 0);
	TESTEQUAL(xattr_size_ret, 0);

	TESTSYSCALL(s_setxattr(s_path(s(mount_dir), s(file_name)), xattr_name,
			       xattr_value, xattr_size, 0));
	TESTEQUAL(bpf_test_trace("setxattr"), 0);

	TESTSYSCALL(s_listxattr(s_path(s(mount_dir), s(file_name)),
				xattr_value_ret, sizeof(xattr_value_ret),
				&xattr_size_ret));
	TESTEQUAL(bpf_test_trace("listxattr"), 0);
	TESTEQUAL(xattr_size_ret, sizeof(xattr_name));
	TESTEQUAL(strcmp(xattr_name, xattr_value_ret), 0);

	TESTSYSCALL(s_getxattr(s_path(s(mount_dir), s(file_name)), xattr_name,
			       xattr_value_ret, sizeof(xattr_value_ret),
			       &xattr_size_ret));
	TESTEQUAL(bpf_test_trace("getxattr"), 0);
	TESTEQUAL(xattr_size, xattr_size_ret);
	TESTEQUAL(strcmp(xattr_value, xattr_value_ret), 0);

	TESTSYSCALL(s_unlink(s_path(s(mount_dir), s(file_name))));
	TESTEQUAL(bpf_test_trace("unlink"), 0);
	TESTEQUAL(s_stat(s_path(s(ft_src), s(file_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_set_backing(const char *mount_dir)
{
	const char *backing_name = "backing";
	const char *test_data = "data";
	const char *test_name = "test";

	int result = TEST_FAILURE;
	int fuse_dev = -1;
	int fd = -1;
	int pid = -1;
	int status;

	TESTEQUAL(mount_fuse_no_init(mount_dir, -1, -1, &fuse_dev), 0);
	FUSE_ACTION
		char data[256] = {0};

		TESTERR(fd = s_open(s_path(s(mount_dir), s(test_name)),
				    O_RDONLY | O_CLOEXEC), fd != -1);
		TESTEQUAL(read(fd, data, strlen(test_data)), strlen(test_data));
		TESTCOND(!strcmp(data, test_data));
		TESTSYSCALL(close(fd));
		fd = -1;
		TESTSYSCALL(umount(mount_dir));
	FUSE_DAEMON
		int bpf_fd  = -1;
		int backing_fd = -1;

		TESTERR(backing_fd = s_creat(s_path(s(ft_src), s(backing_name)), 0777),
			backing_fd != -1);
		TESTEQUAL(write(backing_fd, test_data, strlen(test_data)),
			  strlen(test_data));
		TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_simple",
					  &bpf_fd, NULL, NULL), 0);

		TESTFUSEINIT();
		TESTFUSELOOKUP(test_name, 0);
		TESTFUSEOUT2(fuse_entry_out, ((struct fuse_entry_out) {0}),
			     fuse_entry_bpf_out, ((struct fuse_entry_bpf_out) {
			.backing_action = FUSE_ACTION_REPLACE,
			.backing_fd = backing_fd,
			.bpf_action = FUSE_ACTION_REPLACE,
			.bpf_fd = bpf_fd,
			}));
		read(fuse_dev, bytes_in, sizeof(bytes_in));
		TESTSYSCALL(close(bpf_fd));
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	if (!pid)
		exit(TEST_FAILURE);
	close(fuse_dev);
	close(fd);
	umount(mount_dir);
	return result;
}

static int bpf_test_remove_backing(const char *mount_dir)
{
	const char *folder1 = "folder1";
	const char *folder2 = "folder2";
	const char *file = "file1";
	const char *contents1 = "contents1";
	const char *contents2 = "contents2";

	int result = TEST_FAILURE;
	int fuse_dev = -1;
	int fd = -1;
	int src_fd = -1;
	int bpf_fd = -1;
	int pid = -1;
	int status;
	char data[256] = {0};
	int backing_fd = -1;

	/*
	 * Create folder1/file
	 *        folder2/file
	 *
	 * test will install bpf into mount
	 * bpf will postfilter root lookup to daemon
	 * daemon will remove bpf and redirect opens on folder1 to folder2
	 * test will open folder1/file which will be redirected to folder2
	 * test will check no traces for file, and contents are folder2/file
	 */
	TESTEQUAL(bpf_clear_trace(), 0);
	TESTSYSCALL(s_mkdir(s_path(s(ft_src), s(folder1)), 0777));
	TEST(fd = s_creat(s_pathn(3, s(ft_src), s(folder1), s(file)), 0777),
	     fd != -1);
	TESTEQUAL(write(fd, contents1, strlen(contents1)), strlen(contents1));
	TESTSYSCALL(close(fd));
	TESTSYSCALL(s_mkdir(s_path(s(ft_src), s(folder2)), 0777));
	TEST(fd = s_creat(s_pathn(3, s(ft_src), s(folder2), s(file)), 0777),
	     fd != -1);
	TESTEQUAL(write(fd, contents2, strlen(contents2)), strlen(contents2));
	TESTSYSCALL(close(fd));

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_passthrough", &bpf_fd,
				  NULL, NULL), 0);
	TESTEQUAL(mount_fuse_no_init(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	FUSE_ACTION
		TESTERR(fd = s_open(s_pathn(3, s(mount_dir), s(folder1),
					    s(file)),
				    O_RDONLY | O_CLOEXEC), fd != -1);
		TESTEQUAL(read(fd, data, sizeof(data)), strlen(contents2));
		TESTCOND(!strcmp(data, contents2));
		TESTEQUAL(bpf_test_no_trace("file"), 0);
		TESTSYSCALL(close(fd));
		fd = -1;
		TESTSYSCALL(umount(mount_dir));
	FUSE_DAEMON
		struct {
			char name[8];
			struct fuse_entry_out feo;
			struct fuse_entry_bpf_out febo;
		} __attribute__((packed)) in;

		TESTFUSEINIT();
		TESTFUSEIN(FUSE_LOOKUP | FUSE_POSTFILTER, &in);
		TEST(backing_fd = s_open(s_path(s(ft_src), s(folder2)),
				 O_DIRECTORY | O_RDONLY | O_CLOEXEC),
		     backing_fd != -1);
		TESTFUSEOUT2(fuse_entry_out, ((struct fuse_entry_out) {0}),
			     fuse_entry_bpf_out, ((struct fuse_entry_bpf_out) {
			.bpf_action = FUSE_ACTION_REMOVE,
			.backing_action = FUSE_ACTION_REPLACE,
			.backing_fd = backing_fd,
			}));

		while (read(fuse_dev, bytes_in, sizeof(bytes_in)) != -1) {
			struct fuse_in_header *in_header =
				(struct fuse_in_header *)bytes_in;

			TESTCOND(in_header->opcode == FUSE_FORGET ||
				 in_header->opcode == FUSE_BATCH_FORGET);
		}
	FUSE_DONE

	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	close(fd);
	close(backing_fd);
	close(src_fd);
	close(bpf_fd);
	umount(mount_dir);
	return result;
}

static int bpf_test_dir_rename(const char *mount_dir)
{
	const char *dir_name = "dir";
	const char *dir_name2 = "dir2";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	struct stat st;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_mkdir(s_path(s(mount_dir), s(dir_name)), 0777));
	TESTEQUAL(bpf_test_trace("mkdir"), 0);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(dir_name)), &st));
	TESTSYSCALL(s_rename(s_path(s(mount_dir), s(dir_name)),
			     s_path(s(mount_dir), s(dir_name2))));
	TESTEQUAL(s_stat(s_path(s(ft_src), s(dir_name)), &st), -1);
	TESTEQUAL(errno, ENOENT);
	TESTSYSCALL(s_stat(s_path(s(ft_src), s(dir_name2)), &st));
	result = TEST_SUCCESS;
out:
	close(fuse_dev);
	umount(mount_dir);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int bpf_test_file_rename(const char *mount_dir)
{
	const char *dir = "dir";
	const char *file1 = "file1";
	const char *file2 = "file2";
	int result = TEST_FAILURE;
	int src_fd = -1;
	int bpf_fd = -1;
	int fuse_dev = -1;
	int fd = -1;

	TEST(src_fd = open(ft_src, O_DIRECTORY | O_RDONLY | O_CLOEXEC),
	     src_fd != -1);
	TESTEQUAL(install_elf_bpf("test_bpf.bpf", "test_trace",
				  &bpf_fd, NULL, NULL), 0);
	TESTEQUAL(mount_fuse(mount_dir, bpf_fd, src_fd, &fuse_dev), 0);

	TESTSYSCALL(s_mkdir(s_path(s(mount_dir), s(dir)), 0777));
	TEST(fd = s_creat(s_pathn(3, s(mount_dir), s(dir), s(file1)), 0777),
	     fd != -1);
	TESTSYSCALL(s_rename(s_pathn(3, s(mount_dir), s(dir), s(file1)),
			     s_pathn(3, s(mount_dir), s(dir), s(file2))));
	result = TEST_SUCCESS;
out:
	close(fd);
	umount(mount_dir);
	close(fuse_dev);
	close(bpf_fd);
	close(src_fd);
	return result;
}

static int parse_options(int argc, char *const *argv)
{
	signed char c;

	while ((c = getopt(argc, argv, "f:t:v")) != -1)
		switch (c) {
		case 'f':
			test_options.file = strtol(optarg, NULL, 10);
			break;

		case 't':
			test_options.test = strtol(optarg, NULL, 10);
			break;

		case 'v':
			test_options.verbose = true;
			break;

		default:
			return -EINVAL;
		}

	return 0;
}

struct test_case {
	int (*pfunc)(const char *dir);
	const char *name;
};

static void run_one_test(const char *mount_dir, struct test_case *test_case)
{
	ksft_print_msg("Running %s\n", test_case->name);
	if (test_case->pfunc(mount_dir) == TEST_SUCCESS)
		ksft_test_result_pass("%s\n", test_case->name);
	else
		ksft_test_result_fail("%s\n", test_case->name);
}

int main(int argc, char *argv[])
{
	char *mount_dir = NULL;
	char *src_dir = NULL;
	int i;
	int fd, count;

	if (parse_options(argc, argv))
		ksft_exit_fail_msg("Bad options\n");

	// Seed randomness pool for testing on QEMU
	// NOTE - this abuses the concept of randomness - do *not* ever do this
	// on a machine for production use - the device will think it has good
	// randomness when it does not.
	fd = open("/dev/urandom", O_WRONLY | O_CLOEXEC);
	count = 4096;
	for (int i = 0; i < 128; ++i)
		ioctl(fd, RNDADDTOENTCNT, &count);
	close(fd);

	ksft_print_header();

	if (geteuid() != 0)
		ksft_print_msg("Not a root, might fail to mount.\n");

	if (tracing_on() != TEST_SUCCESS)
		ksft_exit_fail_msg("Can't turn on tracing\n");

	src_dir = setup_mount_dir(ft_src);
	mount_dir = setup_mount_dir(ft_dst);
	if (src_dir == NULL || mount_dir == NULL)
		ksft_exit_fail_msg("Can't create a mount dir\n");

#define MAKE_TEST(test)                                                        \
	{                                                                      \
		test, #test                                                    \
	}
	struct test_case cases[] = {
		MAKE_TEST(basic_test),
		MAKE_TEST(bpf_test_real),
		MAKE_TEST(bpf_test_partial),
		MAKE_TEST(bpf_test_attrs),
		MAKE_TEST(bpf_test_readdir),
		MAKE_TEST(bpf_test_creat),
		MAKE_TEST(bpf_test_hidden_entries),
		MAKE_TEST(bpf_test_dir),
		MAKE_TEST(bpf_test_file_early_close),
		MAKE_TEST(bpf_test_file_late_close),
		MAKE_TEST(bpf_test_mknod),
		MAKE_TEST(bpf_test_largedir),
		MAKE_TEST(bpf_test_link),
		MAKE_TEST(bpf_test_symlink),
		MAKE_TEST(bpf_test_xattr),
		MAKE_TEST(bpf_test_redact_readdir),
		MAKE_TEST(bpf_test_set_backing),
		MAKE_TEST(bpf_test_remove_backing),
		MAKE_TEST(bpf_test_dir_rename),
		MAKE_TEST(bpf_test_file_rename),
		MAKE_TEST(bpf_test_alter_errcode_bpf),
		MAKE_TEST(bpf_test_alter_errcode_userspace),
	};
#undef MAKE_TEST

	if (test_options.test) {
		if (test_options.test <= 0 ||
		    test_options.test > ARRAY_SIZE(cases))
			ksft_exit_fail_msg("Invalid test\n");

		ksft_set_plan(1);
		delete_dir_tree(mount_dir, false);
		delete_dir_tree(src_dir, false);
		run_one_test(mount_dir, &cases[test_options.test - 1]);
	} else {
		ksft_set_plan(ARRAY_SIZE(cases));
		for (i = 0; i < ARRAY_SIZE(cases); ++i) {
			delete_dir_tree(mount_dir, false);
			delete_dir_tree(src_dir, false);
			run_one_test(mount_dir, &cases[i]);
		}
	}

	umount2(mount_dir, MNT_FORCE);
	delete_dir_tree(mount_dir, true);
	delete_dir_tree(src_dir, true);
	return !ksft_get_fail_cnt() ? ksft_exit_pass() : ksft_exit_fail();
}
