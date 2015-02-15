/*
 *  linux/net/sunrpc/clnt.c
 *
 *  This file contains the high-level RPC interface.
 *  It is modeled as a finite state machine to support both synchronous
 *  and asynchronous requests.
 *
 *  -	RPC header generation and argument serialization.
 *  -	Credential refresh.
 *  -	TCP connect handling.
 *  -	Retry of operation when it is suspected the operation failed because
 *	of uid squashing on the server, or when the credentials were stale
 *	and need to be refreshed, or when a packet was damaged in transit.
 *	This may be have to be moved to the VFS layer.
 *
 *  Copyright (C) 1992,1993 Rick Sladkey <jrs@world.std.com>
 *  Copyright (C) 1995,1996 Olaf Kirch <okir@monad.swb.de>
 */


#include <linux/module.h>
#include <linux/types.h>
#include <linux/kallsyms.h>
#include <linux/mm.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/workqueue.h>
#include <linux/in.h>
#include <linux/in6.h>
#include <linux/un.h>
#include <linux/rcupdate.h>

#include <linux/sunrpc/clnt.h>
#include <linux/sunrpc/rpc_pipe_fs.h>
#include <linux/sunrpc/metrics.h>
#include <linux/sunrpc/bc_xprt.h>
#include <trace/events/sunrpc.h>

#include "sunrpc.h"
#include "netns.h"

#ifdef RPC_DEBUG
# define RPCDBG_FACILITY	RPCDBG_CALL
#endif

#define dprint_status(t)					\
	dprintk("RPC: %5u %s (status %d)\n", t->tk_pid,		\
			__func__, t->tk_status)

/*
 * All RPC clients are linked into this list
 */

static DECLARE_WAIT_QUEUE_HEAD(destroy_wait);


static void	call_start(struct rpc_task *task);
static void	call_reserve(struct rpc_task *task);
static void	call_reserveresult(struct rpc_task *task);
static void	call_allocate(struct rpc_task *task);
static void	call_decode(struct rpc_task *task);
static void	call_bind(struct rpc_task *task);
static void	call_bind_status(struct rpc_task *task);
static void	call_transmit(struct rpc_task *task);
#if defined(CONFIG_SUNRPC_BACKCHANNEL)
static void	call_bc_transmit(struct rpc_task *task);
#endif /* CONFIG_SUNRPC_BACKCHANNEL */
static void	call_status(struct rpc_task *task);
static void	call_transmit_status(struct rpc_task *task);
static void	call_refresh(struct rpc_task *task);
static void	call_refreshresult(struct rpc_task *task);
static void	call_timeout(struct rpc_task *task);
static void	call_connect(struct rpc_task *task);
static void	call_connect_status(struct rpc_task *task);

static __be32	*rpc_encode_header(struct rpc_task *task);
static __be32	*rpc_verify_header(struct rpc_task *task);
static int	rpc_ping(struct rpc_clnt *clnt);

static void rpc_register_client(struct rpc_clnt *clnt)
{
	struct net *net = rpc_net_ns(clnt);
	struct sunrpc_net *sn = net_generic(net, sunrpc_net_id);

	spin_lock(&sn->rpc_client_lock);
	list_add(&clnt->cl_clients, &sn->all_clients);
	spin_unlock(&sn->rpc_client_lock);
}

static void rpc_unregister_client(struct rpc_clnt *clnt)
{
	struct net *net = rpc_net_ns(clnt);
	struct sunrpc_net *sn = net_generic(net, sunrpc_net_id);

	spin_lock(&sn->rpc_client_lock);
	list_del(&clnt->cl_clients);
	spin_unlock(&sn->rpc_client_lock);
}

static void __rpc_clnt_remove_pipedir(struct rpc_clnt *clnt)
{
	if (clnt->cl_dentry) {
		if (clnt->cl_auth && clnt->cl_auth->au_ops->pipes_destroy)
			clnt->cl_auth->au_ops->pipes_destroy(clnt->cl_auth);
		rpc_remove_client_dir(clnt->cl_dentry);
	}
	clnt->cl_dentry = NULL;
}

static void rpc_clnt_remove_pipedir(struct rpc_clnt *clnt)
{
	struct net *net = rpc_net_ns(clnt);
	struct super_block *pipefs_sb;

	pipefs_sb = rpc_get_sb_net(net);
	if (pipefs_sb) {
		__rpc_clnt_remove_pipedir(clnt);
		rpc_put_sb_net(net);
	}
}

static struct dentry *rpc_setup_pipedir_sb(struct super_block *sb,
				    struct rpc_clnt *clnt,
				    const char *dir_name)
{
	static uint32_t clntid;
	char name[15];
	struct qstr q = {
		.name = name,
	};
	struct dentry *dir, *dentry;
	int error;

	dir = rpc_d_lookup_sb(sb, dir_name);
	if (dir == NULL)
		return dir;
	for (;;) {
		q.len = snprintf(name, sizeof(name), "clnt%x", (unsigned int)clntid++);
		name[sizeof(name) - 1] = '\0';
		q.hash = full_name_hash(q.name, q.len);
		dentry = rpc_create_client_dir(dir, &q, clnt);
		if (!IS_ERR(dentry))
			break;
		error = PTR_ERR(dentry);
		if (error != -EEXIST) {
			printk(KERN_INFO "RPC: Couldn't create pipefs entry"
					" %s/%s, error %d\n",
					dir_name, name, error);
			break;
		}
	}
	dput(dir);
	return dentry;
}

static int
rpc_setup_pipedir(struct rpc_clnt *clnt, const char *dir_name)
{
	struct net *net = rpc_net_ns(clnt);
	struct super_block *pipefs_sb;
	struct dentry *dentry;

	clnt->cl_dentry = NULL;
	if (dir_name == NULL)
		return 0;
	pipefs_sb = rpc_get_sb_net(net);
	if (!pipefs_sb)
		return 0;
	dentry = rpc_setup_pipedir_sb(pipefs_sb, clnt, dir_name);
	rpc_put_sb_net(net);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);
	clnt->cl_dentry = dentry;
	return 0;
}

static inline int rpc_clnt_skip_event(struct rpc_clnt *clnt, unsigned long event)
{
	if (((event == RPC_PIPEFS_MOUNT) && clnt->cl_dentry) ||
	    ((event == RPC_PIPEFS_UMOUNT) && !clnt->cl_dentry))
		return 1;
	return 0;
}

static int __rpc_clnt_handle_event(struct rpc_clnt *clnt, unsigned long event,
				   struct super_block *sb)
{
	struct dentry *dentry;
	int err = 0;

	switch (event) {
	case RPC_PIPEFS_MOUNT:
		dentry = rpc_setup_pipedir_sb(sb, clnt,
					      clnt->cl_program->pipe_dir_name);
		BUG_ON(dentry == NULL);
		if (IS_ERR(dentry))
			return PTR_ERR(dentry);
		clnt->cl_dentry = dentry;
		if (clnt->cl_auth->au_ops->pipes_create) {
			err = clnt->cl_auth->au_ops->pipes_create(clnt->cl_auth);
			if (err)
				__rpc_clnt_remove_pipedir(clnt);
		}
		break;
	case RPC_PIPEFS_UMOUNT:
		__rpc_clnt_remove_pipedir(clnt);
		break;
	default:
		printk(KERN_ERR "%s: unknown event: %ld\n", __func__, event);
		return -ENOTSUPP;
	}
	return err;
}

static int __rpc_pipefs_event(struct rpc_clnt *clnt, unsigned long event,
				struct super_block *sb)
{
	int error = 0;

	for (;; clnt = clnt->cl_parent) {
		if (!rpc_clnt_skip_event(clnt, event))
			error = __rpc_clnt_handle_event(clnt, event, sb);
		if (error || clnt == clnt->cl_parent)
			break;
	}
	return error;
}

static struct rpc_clnt *rpc_get_client_for_event(struct net *net, int event)
{
	struct sunrpc_net *sn = net_generic(net, sunrpc_net_id);
	struct rpc_clnt *clnt;

	spin_lock(&sn->rpc_client_lock);
	list_for_each_entry(clnt, &sn->all_clients, cl_clients) {
		if (clnt->cl_program->pipe_dir_name == NULL)
			break;
		if (rpc_clnt_skip_event(clnt, event))
			continue;
		if (atomic_inc_not_zero(&clnt->cl_count) == 0)
			continue;
		spin_unlock(&sn->rpc_client_lock);
		return clnt;
	}
	spin_unlock(&sn->rpc_client_lock);
	return NULL;
}

static int rpc_pipefs_event(struct notifier_block *nb, unsigned long event,
			    void *ptr)
{
	struct super_block *sb = ptr;
	struct rpc_clnt *clnt;
	int error = 0;

	while ((clnt = rpc_get_client_for_event(sb->s_fs_info, event))) {
		error = __rpc_pipefs_event(clnt, event, sb);
		rpc_release_client(clnt);
		if (error)
			break;
	}
	return error;
}

static struct notifier_block rpc_clients_block = {
	.notifier_call	= rpc_pipefs_event,
	.priority	= SUNRPC_PIPEFS_RPC_PRIO,
};

int rpc_clients_notifier_register(void)
{
	return rpc_pipefs_notifier_register(&rpc_clients_block);
}

void rpc_clients_notifier_unregister(void)
{
	return rpc_pipefs_notifier_unregister(&rpc_clients_block);
}

static void rpc_clnt_set_nodename(struct rpc_clnt *clnt, const char *nodename)
{
	clnt->cl_nodelen = strlen(nodename);
	if (clnt->cl_nodelen > UNX_MAXNODENAME)
		clnt->cl_nodelen = UNX_MAXNODENAME;
	memcpy(clnt->cl_nodename, nodename, clnt->cl_nodelen);
}

static struct rpc_clnt * rpc_new_client(const struct rpc_create_args *args, struct rpc_xprt *xprt)
{
	const struct rpc_program *program = args->program;
	const struct rpc_version *version;
	struct rpc_clnt		*clnt = NULL;
	struct rpc_auth		*auth;
	int err;

	/* sanity check the name before trying to print it */
	dprintk("RPC:       creating %s client for %s (xprt %p)\n",
			program->name, args->servername, xprt);

	err = rpciod_up();
	if (err)
		goto out_no_rpciod;
	err = -EINVAL;
	if (!xprt)
		goto out_no_xprt;

	if (args->version >= program->nrvers)
		goto out_err;
	version = program->version[args->version];
	if (version == NULL)
		goto out_err;

	err = -ENOMEM;
	clnt = kzalloc(sizeof(*clnt), GFP_KERNEL);
	if (!clnt)
		goto out_err;
	clnt->cl_parent = clnt;

	rcu_assign_pointer(clnt->cl_xprt, xprt);
	clnt->cl_procinfo = version->procs;
	clnt->cl_maxproc  = version->nrprocs;
	clnt->cl_protname = program->name;
	clnt->cl_prog     = args->prognumber ? : program->number;
	clnt->cl_vers     = version->number;
	clnt->cl_stats    = program->stats;
	clnt->cl_metrics  = rpc_alloc_iostats(clnt);
	err = -ENOMEM;
	if (clnt->cl_metrics == NULL)
		goto out_no_stats;
	clnt->cl_program  = program;
	INIT_LIST_HEAD(&clnt->cl_tasks);
	spin_lock_init(&clnt->cl_lock);

	if (!xprt_bound(xprt))
		clnt->cl_autobind = 1;

	clnt->cl_timeout = xprt->timeout;
	if (args->timeout != NULL) {
		memcpy(&clnt->cl_timeout_default, args->timeout,
				sizeof(clnt->cl_timeout_default));
		clnt->cl_timeout = &clnt->cl_timeout_default;
	}

	clnt->cl_rtt = &clnt->cl_rtt_default;
	rpc_init_rtt(&clnt->cl_rtt_default, clnt->cl_timeout->to_initval);
	clnt->cl_principal = NULL;
	if (args->client_name) {
		clnt->cl_principal = kstrdup(args->client_name, GFP_KERNEL);
		if (!clnt->cl_principal)
			goto out_no_principal;
	}

	atomic_set(&clnt->cl_count, 1);

	err = rpc_setup_pipedir(clnt, program->pipe_dir_name);
	if (err < 0)
		goto out_no_path;

	auth = rpcauth_create(args->authflavor, clnt);
	if (IS_ERR(auth)) {
		printk(KERN_INFO "RPC: Couldn't create auth handle (flavor %u)\n",
				args->authflavor);
		err = PTR_ERR(auth);
		goto out_no_auth;
	}

	/* save the nodename */
	rpc_clnt_set_nodename(clnt, utsname()->nodename);
	rpc_register_client(clnt);
	return clnt;

out_no_auth:
	rpc_clnt_remove_pipedir(clnt);
out_no_path:
	kfree(clnt->cl_principal);
out_no_principal:
	rpc_free_iostats(clnt->cl_metrics);
out_no_stats:
	kfree(clnt);
out_err:
	xprt_put(xprt);
out_no_xprt:
	rpciod_down();
out_no_rpciod:
	return ERR_PTR(err);
}

/*
 * rpc_create - create an RPC client and transport with one call
 * @args: rpc_clnt create argument structure
 *
 * Creates and initializes an RPC transport and an RPC client.
 *
 * It can ping the server in order to determine if it is up, and to see if
 * it supports this program and version.  RPC_CLNT_CREATE_NOPING disables
 * this behavior so asynchronous tasks can also use rpc_create.
 */
struct rpc_clnt *rpc_create(struct rpc_create_args *args)
{
	struct rpc_xprt *xprt;
	struct rpc_clnt *clnt;
	struct xprt_create xprtargs = {
		.net = args->net,
		.ident = args->protocol,
		.srcaddr = args->saddress,
		.dstaddr = args->address,
		.addrlen = args->addrsize,
		.servername = args->servername,
		.bc_xprt = args->bc_xprt,
	};
	char servername[48];

	/*
	 * If the caller chooses not to specify a hostname, whip
	 * up a string representation of the passed-in address.
	 */
	if (xprtargs.servername == NULL) {
		struct sockaddr_un *sun =
				(struct sockaddr_un *)args->address;
		struct sockaddr_in *sin =
				(struct sockaddr_in *)args->address;
		struct sockaddr_in6 *sin6 =
				(struct sockaddr_in6 *)args->address;

		servername[0] = '\0';
		switch (args->address->sa_family) {
		case AF_LOCAL:
			snprintf(servername, sizeof(servername), "%s",
				 sun->sun_path);
			break;
		case AF_INET:
			snprintf(servername, sizeof(servername), "%pI4",
				 &sin->sin_addr.s_addr);
			break;
		case AF_INET6:
			snprintf(servername, sizeof(servername), "%pI6",
				 &sin6->sin6_addr);
			break;
		default:
			/* caller wants default server name, but
			 * address family isn't recognized. */
			return ERR_PTR(-EINVAL);
		}
		xprtargs.servername = servername;
	}

	xprt = xprt_create_transport(&xprtargs);
	if (IS_ERR(xprt))
		return (struct rpc_clnt *)xprt;

	/*
	 * By default, kernel RPC client connects from a reserved port.
	 * CAP_NET_BIND_SERVICE will not be set for unprivileged requesters,
	 * but it is always enabled for rpciod, which handles the connect
	 * operation.
	 */
	xprt->resvport = 1;
	if (args->flags & RPC_CLNT_CREATE_NONPRIVPORT)
		xprt->resvport = 0;

	clnt = rpc_new_client(args, xprt);
	if (IS_ERR(clnt))
		return clnt;

	if (!(args->flags & RPC_CLNT_CREATE_NOPING)) {
		int err = rpc_ping(clnt);
		if (err != 0) {
			rpc_shutdown_client(clnt);
			return ERR_PTR(err);
		}
	}

	clnt->cl_softrtry = 1;
	if (args->flags & RPC_CLNT_CREATE_HARDRTRY)
		clnt->cl_softrtry = 0;

	if (args->flags & RPC_CLNT_CREATE_AUTOBIND)
		clnt->cl_autobind = 1;
	if (args->flags & RPC_CLNT_CREATE_DISCRTRY)
		clnt->cl_discrtry = 1;
	if (!(args->flags & RPC_CLNT_CREATE_QUIET))
		clnt->cl_chatty = 1;

	return clnt;
}
EXPORT_SYMBOL_GPL(rpc_create);

/*
 * This function clones the RPC client structure. It allows us to share the
 * same transport while varying parameters such as the authentication
 * flavour.
 */
struct rpc_clnt *
rpc_clone_client(struct rpc_clnt *clnt)
{
	struct rpc_clnt *new;
	struct rpc_xprt *xprt;
	int err = -ENOMEM;

	new = kmemdup(clnt, sizeof(*new), GFP_KERNEL);
	if (!new)
		goto out_no_clnt;
	new->cl_parent = clnt;
	/* Turn off autobind on clones */
	new->cl_autobind = 0;
	INIT_LIST_HEAD(&new->cl_tasks);
	spin_lock_init(&new->cl_lock);
	rpc_init_rtt(&new->cl_rtt_default, clnt->cl_timeout->to_initval);
	new->cl_metrics = rpc_alloc_iostats(clnt);
	if (new->cl_metrics == NULL)
		goto out_no_stats;
	if (clnt->cl_principal) {
		new->cl_principal = kstrdup(clnt->cl_principal, GFP_KERNEL);
		if (new->cl_principal == NULL)
			goto out_no_principal;
	}
	rcu_read_lock();
	xprt = xprt_get(rcu_dereference(clnt->cl_xprt));
	rcu_read_unlock();
	if (xprt == NULL)
		goto out_no_transport;
	rcu_assign_pointer(new->cl_xprt, xprt);
	atomic_set(&new->cl_count, 1);
	err = rpc_setup_pipedir(new, clnt->cl_program->pipe_dir_name);
	if (err != 0)
		goto out_no_path;
	rpc_clnt_set_nodename(new, utsname()->nodename);
	if (new->cl_auth)
		atomic_inc(&new->cl_auth->au_count);
	atomic_inc(&clnt->cl_count);
	rpc_register_client(new);
	rpciod_up();
	return new;
out_no_path:
	xprt_put(xprt);
out_no_transport:
	kfree(new->cl_principal);
out_no_principal:
	rpc_free_iostats(new->cl_metrics);
out_no_stats:
	kfree(new);
out_no_clnt:
	dprintk("RPC:       %s: returned error %d\n", __func__, err);
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(rpc_clone_client);

/*
 * Kill all tasks for the given client.
 * XXX: kill their descendants as well?
 */
void rpc_killall_tasks(struct rpc_clnt *clnt)
{
	struct rpc_task	*rovr;


	if (list_empty(&clnt->cl_tasks))
		return;
	dprintk("RPC:       killing all tasks for client %p\n", clnt);
	/*
	 * Spin lock all_tasks to prevent changes...
	 */
	spin_lock(&clnt->cl_lock);
	list_for_each_entry(rovr, &clnt->cl_tasks, tk_task) {
		if (!RPC_IS_ACTIVATED(rovr))
			continue;
		if (!(rovr->tk_flags & RPC_TASK_KILLED)) {
			rovr->tk_flags |= RPC_TASK_KILLED;
			rpc_exit(rovr, -EIO);
			if (RPC_IS_QUEUED(rovr))
				rpc_wake_up_queued_task(rovr->tk_waitqueue,
							rovr);
		}
	}
	spin_unlock(&clnt->cl_lock);
}
EXPORT_SYMBOL_GPL(rpc_killall_tasks);

/*
 * Properly shut down an RPC client, terminating all outstanding
 * requests.
 */
void rpc_shutdown_client(struct rpc_clnt *clnt)
{
	dprintk_rcu("RPC:       shutting down %s client for %s\n",
			clnt->cl_protname,
			rcu_dereference(clnt->cl_xprt)->servername);

	while (!list_empty(&clnt->cl_tasks)) {
		rpc_killall_tasks(clnt);
		wait_event_timeout(destroy_wait,
			list_empty(&clnt->cl_tasks), 1*HZ);
	}

	rpc_release_client(clnt);
}
EXPORT_SYMBOL_GPL(rpc_shutdown_client);

/*
 * Free an RPC client
 */
static void
rpc_free_client(struct rpc_clnt *clnt)
{
	dprintk_rcu("RPC:       destroying %s client for %s\n",
			clnt->cl_protname,
			rcu_dereference(clnt->cl_xprt)->servername);
	if (clnt->cl_parent != clnt)
		rpc_release_client(clnt->cl_parent);
	rpc_unregister_client(clnt);
	rpc_clnt_remove_pipedir(clnt);
	rpc_free_iostats(clnt->cl_metrics);
	kfree(clnt->cl_principal);
	clnt->cl_metrics = NULL;
	xprt_put(rcu_dereference_raw(clnt->cl_xprt));
	rpciod_down();
	kfree(clnt);
}

/*
 * Free an RPC client
 */
static void
rpc_free_auth(struct rpc_clnt *clnt)
{
	if (clnt->cl_auth == NULL) {
		rpc_free_client(clnt);
		return;
	}

	/*
	 * Note: RPCSEC_GSS may need to send NULL RPC calls in order to
	 *       release remaining GSS contexts. This mechanism ensures
	 *       that it can do so safely.
	 */
	atomic_inc(&clnt->cl_count);
	rpcauth_release(clnt->cl_auth);
	clnt->cl_auth = NULL;
	if (atomic_dec_and_test(&clnt->cl_count))
		rpc_free_client(clnt);
}

/*
 * Release reference to the RPC client
 */
void
rpc_release_client(struct rpc_clnt *clnt)
{
	dprintk("RPC:       rpc_release_client(%p)\n", clnt);

	if (list_empty(&clnt->cl_tasks))
		wake_up(&destroy_wait);
	if (atomic_dec_and_test(&clnt->cl_count))
		rpc_free_auth(clnt);
}

/**
 * rpc_bind_new_program - bind a new RPC program to an existing client
 * @old: old rpc_client
 * @program: rpc program to set
 * @vers: rpc program version
 *
 * Clones the rpc client and sets up a new RPC program. This is mainly
 * of use for enabling different RPC programs to share the same transport.
 * The Sun NFSv2/v3 ACL protocol can do this.
 */
struct rpc_clnt *rpc_bind_new_program(struct rpc_clnt *old,
				      const struct rpc_program *program,
				      u32 vers)
{
	struct rpc_clnt *clnt;
	const struct rpc_version *version;
	int err;

	BUG_ON(vers >= program->nrvers || !program->version[vers]);
	version = program->version[vers];
	clnt = rpc_clone_client(old);
	if (IS_ERR(clnt))
		goto out;
	clnt->cl_procinfo = version->procs;
	clnt->cl_maxproc  = version->nrprocs;
	clnt->cl_protname = program->name;
	clnt->cl_prog     = program->number;
	clnt->cl_vers     = version->number;
	clnt->cl_stats    = program->stats;
	err = rpc_ping(clnt);
	if (err != 0) {
		rpc_shutdown_client(clnt);
		clnt = ERR_PTR(err);
	}
out:
	return clnt;
}
EXPORT_SYMBOL_GPL(rpc_bind_new_program);

void rpc_task_release_client(struct rpc_task *task)
{
	struct rpc_clnt *clnt = task->tk_client;

	if (clnt != NULL) {
		/* Remove from client task list */
		spin_lock(&clnt->cl_lock);
		list_del(&task->tk_task);
		spin_unlock(&clnt->cl_lock);
		task->tk_client = NULL;

		rpc_release_client(clnt);
	}
}

static
void rpc_task_set_client(struct rpc_task *task, struct rpc_clnt *clnt)
{
	if (clnt != NULL) {
		rpc_task_release_client(task);
		task->tk_client = clnt;
		atomic_inc(&clnt->cl_count);
		if (clnt->cl_softrtry)
			task->tk_flags |= RPC_TASK_SOFT;
		/* Add to the client's list of all tasks */
		spin_lock(&clnt->cl_lock);
		list_add_tail(&task->tk_task, &clnt->cl_tasks);
		spin_unlock(&clnt->cl_lock);
	}
}

void rpc_task_reset_client(struct rpc_task *task, struct rpc_clnt *clnt)
{
	rpc_task_release_client(task);
	rpc_task_set_client(task, clnt);
}
EXPORT_SYMBOL_GPL(rpc_task_reset_client);


static void
rpc_task_set_rpc_message(struct rpc_task *task, const struct rpc_message *msg)
{
	if (msg != NULL) {
		task->tk_msg.rpc_proc = msg->rpc_proc;
		task->tk_msg.rpc_argp = msg->rpc_argp;
		task->tk_msg.rpc_resp = msg->rpc_resp;
		if (msg->rpc_cred != NULL)
			task->tk_msg.rpc_cred = get_rpccred(msg->rpc_cred);
	}
}

/*
 * Default callback for async RPC calls
 */
static void
rpc_default_callback(struct rpc_task *task, void *data)
{
}

static const struct rpc_call_ops rpc_default_ops = {
	.rpc_call_done = rpc_default_callback,
};

/**
 * rpc_run_task - Allocate a new RPC task, then run rpc_execute against it
 * @task_setup_data: pointer to task initialisation data
 */
struct rpc_task *rpc_run_task(const struct rpc_task_setup *task_setup_data)
{
	struct rpc_task *task;

	task = rpc_new_task(task_setup_data);
	if (IS_ERR(task))
		goto out;

	rpc_task_set_client(task, task_setup_data->rpc_client);
	rpc_task_set_rpc_message(task, task_setup_data->rpc_message);

	if (task->tk_action == NULL)
		rpc_call_start(task);

	atomic_inc(&task->tk_count);
	rpc_execute(task);
out:
	return task;
}
EXPORT_SYMBOL_GPL(rpc_run_task);

/**
 * rpc_call_sync - Perform a synchronous RPC call
 * @clnt: pointer to RPC client
 * @msg: RPC call parameters
 * @flags: RPC call flags
 */
int rpc_call_sync(struct rpc_clnt *clnt, const struct rpc_message *msg, int flags)
{
	struct rpc_task	*task;
	struct rpc_task_setup task_setup_data = {
		.rpc_client = clnt,
		.rpc_message = msg,
		.callback_ops = &rpc_default_ops,
		.flags = flags,
	};
	int status;

	BUG_ON(flags & RPC_TASK_ASYNC);

	task = rpc_run_task(&task_setup_data);
	if (IS_ERR(task))
		return PTR_ERR(task);
	status = task->tk_status;
	rpc_put_task(task);
	return status;
}
EXPORT_SYMBOL_GPL(rpc_call_sync);

/**
 * rpc_call_async - Perform an asynchronous RPC call
 * @clnt: pointer to RPC client
 * @msg: RPC call parameters
 * @flags: RPC call flags
 * @tk_ops: RPC call ops
 * @data: user call data
 */
int
rpc_call_async(struct rpc_clnt *clnt, const struct rpc_message *msg, int flags,
	       const struct rpc_call_ops *tk_ops, void *data)
{
	struct rpc_task	*task;
	struct rpc_task_setup task_setup_data = {
		.rpc_client = clnt,
		.rpc_message = msg,
		.callback_ops = tk_ops,
		.callback_data = data,
		.flags = flags|RPC_TASK_ASYNC,
	};

	task = rpc_run_task(&task_setup_data);
	if (IS_ERR(task))
		return PTR_ERR(task);
	rpc_put_task(task);
	return 0;
}
EXPORT_SYMBOL_GPL(rpc_call_async);

#if defined(CONFIG_SUNRPC_BACKCHANNEL)
/**
 * rpc_run_bc_task - Allocate a new RPC task for backchannel use, then run
 * rpc_execute against it
 * @req: RPC request
 * @tk_ops: RPC call ops
 */
struct rpc_task *rpc_run_bc_task(struct rpc_rqst *req,
				const struct rpc_call_ops *tk_ops)
{
	struct rpc_task *task;
	struct xdr_buf *xbufp = &req->rq_snd_buf;
	struct rpc_task_setup task_setup_data = {
		.callback_ops = tk_ops,
	};

	dprintk("RPC: rpc_run_bc_task req= %p\n", req);
	/*
	 * Create an rpc_task to send the data
	 */
	task = rpc_new_task(&task_setup_data);
	if (IS_ERR(task)) {
		xprt_free_bc_request(req);
		goto out;
	}
	task->tk_rqstp = req;

	/*
	 * Set up the xdr_buf length.
	 * This also indicates that the buffer is XDR encoded already.
	 */
	xbufp->len = xbufp->head[0].iov_len + xbufp->page_len +
			xbufp->tail[0].iov_len;

	task->tk_action = call_bc_transmit;
	atomic_inc(&task->tk_count);
	BUG_ON(atomic_read(&task->tk_count) != 2);
	rpc_execute(task);

out:
	dprintk("RPC: rpc_run_bc_task: task= %p\n", task);
	return task;
}
#endif /* CONFIG_SUNRPC_BACKCHANNEL */

void
rpc_call_start(struct rpc_task *task)
{
	task->tk_action = call_start;
}
EXPORT_SYMBOL_GPL(rpc_call_start);

/**
 * rpc_peeraddr - extract remote peer address from clnt's xprt
 * @clnt: RPC client structure
 * @buf: target buffer
 * @bufsize: length of target buffer
 *
 * Returns the number of bytes that are actually in the stored address.
 */
size_t rpc_peeraddr(struct rpc_clnt *clnt, struct sockaddr *buf, size_t bufsize)
{
	size_t bytes;
	struct rpc_xprt *xprt;

	rcu_read_lock();
	xprt = rcu_dereference(clnt->cl_xprt);

	bytes = xprt->addrlen;
	if (bytes > bufsize)
		bytes = bufsize;
	memcpy(buf, &xprt->addr, bytes);
	rcu_read_unlock();

	return bytes;
}
EXPORT_SYMBOL_GPL(rpc_peeraddr);

/**
 * rpc_peeraddr2str - return remote peer address in printable format
 * @clnt: RPC client structure
 * @format: address format
 *
 * NB: the lifetime of the memory referenced by the returned pointer is
 * the same as the rpc_xprt itself.  As long as the caller uses this
 * pointer, it must hold the RCU read lock.
 */
const char *rpc_peeraddr2str(struct rpc_clnt *clnt,
			     enum rpc_display_format_t format)
{
	struct rpc_xprt *xprt;

	xprt = rcu_dereference(clnt->cl_xprt);

	if (xprt->address_strings[format] != NULL)
		return xprt->address_strings[format];
	else
		return "unprintable";
}
EXPORT_SYMBOL_GPL(rpc_peeraddr2str);

static const struct sockaddr_in rpc_inaddr_loopback = {
	.sin_family		= AF_INET,
	.sin_addr.s_addr	= htonl(INADDR_ANY),
};

static const struct sockaddr_in6 rpc_in6addr_loopback = {
	.sin6_family		= AF_INET6,
	.sin6_addr		= IN6ADDR_ANY_INIT,
};

/*
 * Try a getsockname() on a connected datagram socket.  Using a
 * connected datagram socket prevents leaving a socket in TIME_WAIT.
 * This conserves the ephemeral port number space.
 *
 * Returns zero and fills in "buf" if successful; otherwise, a
 * negative errno is returned.
 */
static int rpc_sockname(struct net *net, struct sockaddr *sap, size_t salen,
			struct sockaddr *buf, int buflen)
{
	struct socket *sock;
	int err;

	err = __sock_create(net, sap->sa_family,
				SOCK_DGRAM, IPPROTO_UDP, &sock, 1);
	if (err < 0) {
		dprintk("RPC:       can't create UDP socket (%d)\n", err);
		goto out;
	}

	switch (sap->sa_family) {
	case AF_INET:
		err = kernel_bind(sock,
				(struct sockaddr *)&rpc_inaddr_loopback,
				sizeof(rpc_inaddr_loopback));
		break;
	case AF_INET6:
		err = kernel_bind(sock,
				(struct sockaddr *)&rpc_in6addr_loopback,
				sizeof(rpc_in6addr_loopback));
		break;
	default:
		err = -EAFNOSUPPORT;
		goto out;
	}
	if (err < 0) {
		dprintk("RPC:       can't bind UDP socket (%d)\n", err);
		goto out_release;
	}

	err = kernel_connect(sock, sap, salen, 0);
	if (err < 0) {
		dprintk("RPC:       can't connect UDP socket (%d)\n", err);
		goto out_release;
	}

	err = kernel_getsockname(sock, buf, &buflen);
	if (err < 0) {
		dprintk("RPC:       getsockname failed (%d)\n", err);
		goto out_release;
	}

	err = 0;
	if (buf->sa_family == AF_INET6) {
		struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)buf;
		sin6->sin6_scope_id = 0;
	}
	dprintk("RPC:       %s succeeded\n", __func__);

out_release:
	sock_release(sock);
out:
	return err;
}

/*
 * Scraping a connected socket failed, so we don't have a useable
 * local address.  Fallback: generate an address that will prevent
 * the server from calling us back.
 *
 * Returns zero and fills in "buf" if successful; otherwise, a
 * negative errno is returned.
 */
static int rpc_anyaddr(int family, struct sockaddr *buf, size_t buflen)
{
	switch (family) {
	case AF_INET:
		if (buflen < sizeof(rpc_inaddr_loopback))
			return -EINVAL;
		memcpy(buf, &rpc_inaddr_loopback,
				sizeof(rpc_inaddr_loopback));
		break;
	case AF_INET6:
		if (buflen < sizeof(rpc_in6addr_loopback))
			return -EINVAL;
		memcpy(buf, &rpc_in6addr_loopback,
				sizeof(rpc_in6addr_loopback));
	default:
		dprintk("RPC:       %s: address family not supported\n",
			__func__);
		return -EAFNOSUPPORT;
	}
	dprintk("RPC:       %s: succeeded\n", __func__);
	return 0;
}

/**
 * rpc_localaddr - discover local endpoint address for an RPC client
 * @clnt: RPC client structure
 * @buf: target buffer
 * @buflen: size of target buffer, in bytes
 *
 * Returns zero and fills in "buf" and "buflen" if successful;
 * otherwise, a negative errno is returned.
 *
 * This works even if the underlying transport is not currently connected,
 * or if the upper layer never previously provided a source address.
 *
 * The result of this function call is transient: multiple calls in
 * succession may give different results, depending on how local
 * networking configuration changes over time.
 */
int rpc_localaddr(struct rpc_clnt *clnt, struct sockaddr *buf, size_t buflen)
{
	struct sockaddr_storage address;
	struct sockaddr *sap = (struct sockaddr *)&address;
	struct rpc_xprt *xprt;
	struct net *net;
	size_t salen;
	int err;

	rcu_read_lock();
	xprt = rcu_dereference(clnt->cl_xprt);
	salen = xprt->addrlen;
	memcpy(sap, &xprt->addr, salen);
	net = get_net(xprt->xprt_net);
	rcu_read_unlock();

	rpc_set_port(sap, 0);
	err = rpc_sockname(net, sap, salen, buf, buflen);
	put_net(net);
	if (err != 0)
		/* Couldn't discover local address, return ANYADDR */
		return rpc_anyaddr(sap->sa_family, buf, buflen);
	return 0;
}
EXPORT_SYMBOL_GPL(rpc_localaddr);

void
rpc_setbufsize(struct rpc_clnt *clnt, unsigned int sndsize, unsigned int rcvsize)
{
	struct rpc_xprt *xprt;

	rcu_read_lock();
	xprt = rcu_dereference(clnt->cl_xprt);
	if (xprt->ops->set_buffer_size)
		xprt->ops->set_buffer_size(xprt, sndsize, rcvsize);
	rcu_read_unlock();
}
EXPORT_SYMBOL_GPL(rpc_setbufsize);

/**
 * rpc_protocol - Get transport protocol number for an RPC client
 * @clnt: RPC client to query
 *
 */
int rpc_protocol(struct rpc_clnt *clnt)
{
	int protocol;

	rcu_read_lock();
	protocol = rcu_dereference(clnt->cl_xprt)->prot;
	rcu_read_unlock();
	return protocol;
}
EXPORT_SYMBOL_GPL(rpc_protocol);

/**
 * rpc_net_ns - Get the network namespace for this RPC client
 * @clnt: RPC client to query
 *
 */
struct net *rpc_net_ns(struct rpc_clnt *clnt)
{
	struct net *ret;

	rcu_read_lock();
	ret = rcu_dereference(clnt->cl_xprt)->xprt_net;
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(rpc_net_ns);

/**
 * rpc_max_payload - Get maximum payload size for a transport, in bytes
 * @clnt: RPC client to query
 *
 * For stream transports, this is one RPC record fragment (see RFC
 * 1831), as we don't support multi-record requests yet.  For datagram
 * transports, this is the size of an IP packet minus the IP, UDP, and
 * RPC header sizes.
 */
size_t rpc_max_payload(struct rpc_clnt *clnt)
{
	size_t ret;

	rcu_read_lock();
	ret = rcu_dereference(clnt->cl_xprt)->max_payload;
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(rpc_max_payload);

/**
 * rpc_force_rebind - force transport to check that remote port is unchanged
 * @clnt: client to rebind
 *
 */
void rpc_force_rebind(struct rpc_clnt *clnt)
{
	if (clnt->cl_autobind) {
		rcu_read_lock();
		xprt_clear_bound(rcu_dereference(clnt->cl_xprt));
		rcu_read_unlock();
	}
}
EXPORT_SYMBOL_GPL(rpc_force_rebind);

/*
 * Restart an (async) RPC call from the call_prepare state.
 * Usually called from within the exit handler.
 */
int
rpc_restart_call_prepare(struct rpc_task *task)
{
	if (RPC_ASSASSINATED(task))
		return 0;
	task->tk_action = call_start;
	if (task->tk_ops->rpc_call_prepare != NULL)
		task->tk_action = rpc_prepare_task;
	return 1;
}
EXPORT_SYMBOL_GPL(rpc_restart_call_prepare);

/*
 * Restart an (async) RPC call. Usually called from within the
 * exit handler.
 */
int
rpc_restart_call(struct rpc_task *task)
{
	if (RPC_ASSASSINATED(task))
		return 0;
	task->tk_action = call_start;
	return 1;
}
EXPORT_SYMBOL_GPL(rpc_restart_call);

#ifdef RPC_DEBUG
static const char *rpc_proc_name(const struct rpc_task *task)
{
	const struct rpc_procinfo *proc = task->tk_msg.rpc_proc;

	if (proc) {
		if (proc->p_name)
			return proc->p_name;
		else
			return "NULL";
	} else
		return "no proc";
}
#endif

/*
 * 0.  Initial state
 *
 *     Other FSM states can be visited zero or more times, but
 *     this state is visited exactly once for each RPC.
 */
static void
call_start(struct rpc_task *task)
{
	struct rpc_clnt	*clnt = task->tk_client;

	dprintk("RPC: %5u call_start %s%d proc %s (%s)\n", task->tk_pid,
			clnt->cl_protname, clnt->cl_vers,
			rpc_proc_name(task),
			(RPC_IS_ASYNC(task) ? "async" : "sync"));

	/* Increment call count */
	task->tk_msg.rpc_proc->p_count++;
	clnt->cl_stats->rpccnt++;
	task->tk_action = call_reserve;
}

/*
 * 1.	Reserve an RPC call slot
 */
static void
call_reserve(struct rpc_task *task)
{
	dprint_status(task);

	task->tk_status  = 0;
	task->tk_action  = call_reserveresult;
	xprt_reserve(task);
}

/*
 * 1b.	Grok the result of xprt_reserve()
 */
static void
call_reserveresult(struct rpc_task *task)
{
	int status = task->tk_status;

	dprint_status(task);

	/*
	 * After a call to xprt_reserve(), we must have either
	 * a request slot or else an error status.
	 */
	task->tk_status = 0;
	if (status >= 0) {
		if (task->tk_rqstp) {
			task->tk_action = call_refresh;
			return;
		}

		printk(KERN_ERR "%s: status=%d, but no request slot, exiting\n",
				__func__, status);
		rpc_exit(task, -EIO);
		return;
	}

	/*
	 * Even though there was an error, we may have acquired
	 * a request slot somehow.  Make sure not to leak it.
	 */
	if (task->tk_rqstp) {
		printk(KERN_ERR "%s: status=%d, request allocated anyway\n",
				__func__, status);
		xprt_release(task);
	}

	switch (status) {
	case -EAGAIN:	/* woken up; retry */
		task->tk_action = call_reserve;
		return;
	case -EIO:	/* probably a shutdown */
		break;
	default:
		printk(KERN_ERR "%s: unrecognized error %d, exiting\n",
				__func__, status);
		break;
	}
	rpc_exit(task, status);
}

/*
 * 2.	Bind and/or refresh the credentials
 */
static void
call_refresh(struct rpc_task *task)
{
	dprint_status(task);

	task->tk_action = call_refreshresult;
	task->tk_status = 0;
	task->tk_client->cl_stats->rpcauthrefresh++;
	rpcauth_refreshcred(task);
}

/*
 * 2a.	Process the results of a credential refresh
 */
static void
call_refreshresult(struct rpc_task *task)
{
	int status = task->tk_status;

	dprint_status(task);

	task->tk_status = 0;
	task->tk_action = call_refresh;
	switch (status) {
	case 0:
		if (rpcauth_uptodatecred(task))
			task->tk_action = call_allocate;
		return;
	case -ETIMEDOUT:
		rpc_delay(task, 3*HZ);
	case -EAGAIN:
		status = -EACCES;
		if (!task->tk_cred_retry)
			break;
		task->tk_cred_retry--;
		dprintk("RPC: %5u %s: retry refresh creds\n",
				task->tk_pid, __func__);
		return;
	}
	dprintk("RPC: %5u %s: refresh creds failed with error %d\n",
				task->tk_pid, __func__, status);
	rpc_exit(task, status);
}

/*
 * 2b.	Allocate the buffer. For details, see sched.c:rpc_malloc.
 *	(Note: buffer memory is freed in xprt_release).
 */
static void
call_allocate(struct rpc_task *task)
{
	unsigned int slack = task->tk_rqstp->rq_cred->cr_auth->au_cslack;
	struct rpc_rqst *req = task->tk_rqstp;
	struct rpc_xprt *xprt = task->tk_xprt;
	struct rpc_procinfo *proc = task->tk_msg.rpc_proc;

	dprint_status(task);

	task->tk_status = 0;
	task->tk_action = call_bind;

	if (req->rq_buffer)
		return;

	if (proc->p_proc != 0) {
		BUG_ON(proc->p_arglen == 0);
		if (proc->p_decode != NULL)
			BUG_ON(proc->p_replen == 0);
	}

	/*
	 * Calculate the size (in quads) of the RPC call
	 * and reply headers, and convert both values
	 * to byte sizes.
	 */
	req->rq_callsize = RPC_CALLHDRSIZE + (slack << 1) + proc->p_arglen;
	req->rq_callsize <<= 2;
	req->rq_rcvsize = RPC_REPHDRSIZE + slack + proc->p_replen;
	req->rq_rcvsize <<= 2;

	req->rq_buffer = xprt->ops->buf_alloc(task,
					req->rq_callsize + req->rq_rcvsize);
	if (req->rq_buffer != NULL)
		return;

	dprintk("RPC: %5u rpc_buffer allocation failed\n", task->tk_pid);

	if (RPC_IS_ASYNC(task) || !fatal_signal_pending(current)) {
		task->tk_action = call_allocate;
		rpc_delay(task, HZ>>4);
		return;
	}

	rpc_exit(task, -ERESTARTSYS);
}

static inline int
rpc_task_need_encode(struct rpc_task *task)
{
	return task->tk_rqstp->rq_snd_buf.len == 0;
}

static inline void
rpc_task_force_reencode(struct rpc_task *task)
{
	task->tk_rqstp->rq_snd_buf.len = 0;
	task->tk_rqstp->rq_bytes_sent = 0;
}

static inline void
rpc_xdr_buf_init(struct xdr_buf *buf, void *start, size_t len)
{
	buf->head[0].iov_base = start;
	buf->head[0].iov_len = len;
	buf->tail[0].iov_len = 0;
	buf->page_len = 0;
	buf->flags = 0;
	buf->len = 0;
	buf->buflen = len;
}

/*
 * 3.	Encode arguments of an RPC call
 */
static void
rpc_xdr_encode(struct rpc_task *task)
{
	struct rpc_rqst	*req = task->tk_rqstp;
	kxdreproc_t	encode;
	__be32		*p;

	dprint_status(task);

	rpc_xdr_buf_init(&req->rq_snd_buf,
			 req->rq_buffer,
			 req->rq_callsize);
	rpc_xdr_buf_init(&req->rq_rcv_buf,
			 (char *)req->rq_buffer + req->rq_callsize,
			 req->rq_rcvsize);

	p = rpc_encode_header(task);
	if (p == NULL) {
		printk(KERN_INFO "RPC: couldn't encode RPC header, exit EIO\n");
		rpc_exit(task, -EIO);
		return;
	}

	encode = task->tk_msg.rpc_proc->p_encode;
	if (encode == NULL)
		return;

	task->tk_status = rpcauth_wrap_req(task, encode, req, p,
			task->tk_msg.rpc_argp);
}

/*
 * 4.	Get the server port number if not yet set
 */
static void
call_bind(struct rpc_task *task)
{
	struct rpc_xprt *xprt = task->tk_xprt;

	dprint_status(task);

	task->tk_action = call_connect;
	if (!xprt_bound(xprt)) {
		task->tk_action = call_bind_status;
		task->tk_timeout = xprt->bind_timeout;
		xprt->ops->rpcbind(task);
	}
}

/*
 * 4a.	Sort out bind result
 */
static void
call_bind_status(struct rpc_task *task)
{
	int status = -EIO;

	if (task->tk_status >= 0) {
		dprint_status(task);
		task->tk_status = 0;
		task->tk_action = call_connect;
		return;
	}

	trace_rpc_bind_status(task);
	switch (task->tk_status) {
	case -ENOMEM:
		dprintk("RPC: %5u rpcbind out of memory\n", task->tk_pid);
		rpc_delay(task, HZ >> 2);
		goto retry_timeout;
	case -EACCES:
		dprintk("RPC: %5u remote rpcbind: RPC program/version "
				"unavailable\n", task->tk_pid);
		/* fail immediately if this is an RPC ping */
		if (task->tk_msg.rpc_proc->p_proc == 0) {
			status = -EOPNOTSUPP;
			break;
		}
		if (task->tk_rebind_retry == 0)
			break;
		task->tk_rebind_retry--;
		rpc_delay(task, 3*HZ);
		goto retry_timeout;
	case -ETIMEDOUT:
		dprintk("RPC: %5u rpcbind request timed out\n",
				task->tk_pid);
		goto retry_timeout;
	case -EPFNOSUPPORT:
		/* server doesn't support any rpcbind version we know of */
		dprintk("RPC: %5u unrecognized remote rpcbind service\n",
				task->tk_pid);
		break;
	case -EPROTONOSUPPORT:
		dprintk("RPC: %5u remote rpcbind version unavailable, retrying\n",
				task->tk_pid);
		task->tk_status = 0;
		task->tk_action = call_bind;
		return;
	case -ECONNREFUSED:		/* connection problems */
	case -ECONNRESET:
	case -ENOTCONN:
	case -EHOSTDOWN:
	case -EHOSTUNREACH:
	case -ENETUNREACH:
	case -EPIPE:
		dprintk("RPC: %5u remote rpcbind unreachable: %d\n",
				task->tk_pid, task->tk_status);
		if (!RPC_IS_SOFTCONN(task)) {
			rpc_delay(task, 5*HZ);
			goto retry_timeout;
		}
		status = task->tk_status;
		break;
	default:
		dprintk("RPC: %5u unrecognized rpcbind error (%d)\n",
				task->tk_pid, -task->tk_status);
	}

	rpc_exit(task, status);
	return;

retry_timeout:
	task->tk_action = call_timeout;
}

/*
 * 4b.	Connect to the RPC server
 */
static void
call_connect(struct rpc_task *task)
{
	struct rpc_xprt *xprt = task->tk_xprt;

	dprintk("RPC: %5u call_connect xprt %p %s connected\n",
			task->tk_pid, xprt,
			(xprt_connected(xprt) ? "is" : "is not"));

	task->tk_action = call_transmit;
	if (!xprt_connected(xprt)) {
		task->tk_action = call_connect_status;
		if (task->tk_status < 0)
			return;
		xprt_connect(task);
	}
}

/*
 * 4c.	Sort out connect result
 */
static void
call_connect_status(struct rpc_task *task)
{
	struct rpc_clnt *clnt = task->tk_client;
	int status = task->tk_status;

	dprint_status(task);

	task->tk_status = 0;
	if (status >= 0 || status == -EAGAIN) {
		clnt->cl_stats->netreconn++;
		task->tk_action = call_transmit;
		return;
	}

	trace_rpc_connect_status(task, status);
	switch (status) {
		/* if soft mounted, test if we've timed out */
	case -ETIMEDOUT:
		task->tk_action = call_timeout;
		break;
	default:
		rpc_exit(task, -EIO);
	}
}

/*
 * 5.	Transmit the RPC request, and wait for reply
 */
static void
call_transmit(struct rpc_task *task)
{
	dprint_status(task);

	task->tk_action = call_status;
	if (task->tk_status < 0)
		return;
	task->tk_status = xprt_prepare_transmit(task);
	if (task->tk_status != 0)
		return;
	task->tk_action = call_transmit_status;
	/* Encode here so that rpcsec_gss can use correct sequence number. */
	if (rpc_task_need_encode(task)) {
		BUG_ON(task->tk_rqstp->rq_bytes_sent != 0);
		rpc_xdr_encode(task);
		/* Did the encode result in an error condition? */
		if (task->tk_status != 0) {
			/* Was the error nonfatal? */
			if (task->tk_status == -EAGAIN)
				rpc_delay(task, HZ >> 4);
			else
				rpc_exit(task, task->tk_status);
			return;
		}
	}
	xprt_transmit(task);
	if (task->tk_status < 0)
		return;
	/*
	 * On success, ensure that we call xprt_end_transmit() before sleeping
	 * in order to allow access to the socket to other RPC requests.
	 */
	call_transmit_status(task);
	if (rpc_reply_expected(task))
		return;
	task->tk_action = rpc_exit_task;
	rpc_wake_up_queued_task(&task->tk_xprt->pending, task);
}

/*
 * 5a.	Handle cleanup after a transmission
 */
static void
call_transmit_status(struct rpc_task *task)
{
	task->tk_action = call_status;

	/*
	 * Common case: success.  Force the compiler to put this
	 * test first.
	 */
	if (task->tk_status == 0) {
		xprt_end_transmit(task);
		rpc_task_force_reencode(task);
		return;
	}

	switch (task->tk_status) {
	case -EAGAIN:
		break;
	default:
		dprint_status(task);
		xprt_end_transmit(task);
		rpc_task_force_reencode(task);
		break;
		/*
		 * Special cases: if we've been waiting on the
		 * socket's write_space() callback, or if the
		 * socket just returned a connection error,
		 * then hold onto the transport lock.
		 */
	case -ECONNREFUSED:
	case -EHOSTDOWN:
	case -EHOSTUNREACH:
	case -ENETUNREACH:
		if (RPC_IS_SOFTCONN(task)) {
			xprt_end_transmit(task);
			rpc_exit(task, task->tk_status);
			break;
		}
	case -ECONNRESET:
	case -ENOTCONN:
	case -EPIPE:
		rpc_task_force_reencode(task);
	}
}

#if defined(CONFIG_SUNRPC_BACKCHANNEL)
/*
 * 5b.	Send the backchannel RPC reply.  On error, drop the reply.  In
 * addition, disconnect on connectivity errors.
 */
static void
call_bc_transmit(struct rpc_task *task)
{
	struct rpc_rqst *req = task->tk_rqstp;

	BUG_ON(task->tk_status != 0);
	task->tk_status = xprt_prepare_transmit(task);
	if (task->tk_status == -EAGAIN) {
		/*
		 * Could not reserve the transport. Try again after the
		 * transport is released.
		 */
		task->tk_status = 0;
		task->tk_action = call_bc_transmit;
		return;
	}

	task->tk_action = rpc_exit_task;
	if (task->tk_status < 0) {
		printk(KERN_NOTICE "RPC: Could not send backchannel reply "
			"error: %d\n", task->tk_status);
		return;
	}

	xprt_transmit(task);
	xprt_end_transmit(task);
	dprint_status(task);
	switch (task->tk_status) {
	case 0:
		/* Success */
		break;
	case -EHOSTDOWN:
	case -EHOSTUNREACH:
	case -ENETUNREACH:
	case -ETIMEDOUT:
		/*
		 * Problem reaching the server.  Disconnect and let the
		 * forechannel reestablish the connection.  The server will
		 * have to retransmit the backchannel request and we'll
		 * reprocess it.  Since these ops are idempotent, there's no
		 * need to cache our reply at this time.
		 */
		printk(KERN_NOTICE "RPC: Could not send backchannel reply "
			"error: %d\n", task->tk_status);
		xprt_conditional_disconnect(task->tk_xprt,
			req->rq_connect_cookie);
		break;
	default:
		/*
		 * We were unable to reply and will have to drop the
		 * request.  The server should reconnect and retransmit.
		 */
		BUG_ON(task->tk_status == -EAGAIN);
		printk(KERN_NOTICE "RPC: Could not send backchannel reply "
			"error: %d\n", task->tk_status);
		break;
	}
	rpc_wake_up_queued_task(&req->rq_xprt->pending, task);
}
#endif /* CONFIG_SUNRPC_BACKCHANNEL */

/*
 * 6.	Sort out the RPC call status
 */
static void
call_status(struct rpc_task *task)
{
	struct rpc_clnt	*clnt = task->tk_client;
	struct rpc_rqst	*req = task->tk_rqstp;
	int		status;

	if (req->rq_reply_bytes_recvd > 0 && !req->rq_bytes_sent)
		task->tk_status = req->rq_reply_bytes_recvd;

	dprint_status(task);

	status = task->tk_status;
	if (status >= 0) {
		task->tk_action = call_decode;
		return;
	}

	trace_rpc_call_status(task);
	task->tk_status = 0;
	switch(status) {
	case -EHOSTDOWN:
	case -EHOSTUNREACH:
	case -ENETUNREACH:
		/*
		 * Delay any retries for 3 seconds, then handle as if it
		 * were a timeout.
		 */
		rpc_delay(task, 3*HZ);
	case -ETIMEDOUT:
		task->tk_action = call_timeout;
		if (task->tk_client->cl_discrtry)
			xprt_conditional_disconnect(task->tk_xprt,
					req->rq_connect_cookie);
		break;
	case -ECONNRESET:
	case -ECONNREFUSED:
		rpc_force_rebind(clnt);
		rpc_delay(task, 3*HZ);
	case -EPIPE:
	case -ENOTCONN:
		task->tk_action = call_bind;
		break;
	case -EAGAIN:
		task->tk_action = call_transmit;
		break;
	case -EIO:
		/* shutdown or soft timeout */
		rpc_exit(task, status);
		break;
	default:
		if (clnt->cl_chatty)
			printk("%s: RPC call returned error %d\n",
			       clnt->cl_protname, -status);
		rpc_exit(task, status);
	}
}

/*
 * 6a.	Handle RPC timeout
 * 	We do not release the request slot, so we keep using the
 *	same XID for all retransmits.
 */
static void
call_timeout(struct rpc_task *task)
{
	struct rpc_clnt	*clnt = task->tk_client;

	if (xprt_adjust_timeout(task->tk_rqstp) == 0) {
		dprintk("RPC: %5u call_timeout (minor)\n", task->tk_pid);
		goto retry;
	}

	dprintk("RPC: %5u call_timeout (major)\n", task->tk_pid);
	task->tk_timeouts++;

	if (RPC_IS_SOFTCONN(task)) {
		rpc_exit(task, -ETIMEDOUT);
		return;
	}
	if (RPC_IS_SOFT(task)) {
		if (clnt->cl_chatty)
			rcu_read_lock();
			printk(KERN_NOTICE "%s: server %s not responding, timed out\n",
				clnt->cl_protname,
				rcu_dereference(clnt->cl_xprt)->servername);
			rcu_read_unlock();
		if (task->tk_flags & RPC_TASK_TIMEOUT)
			rpc_exit(task, -ETIMEDOUT);
		else
			rpc_exit(task, -EIO);
		return;
	}

	if (!(task->tk_flags & RPC_CALL_MAJORSEEN)) {
		task->tk_flags |= RPC_CALL_MAJORSEEN;
		if (clnt->cl_chatty) {
			rcu_read_lock();
			printk(KERN_NOTICE "%s: server %s not responding, still trying\n",
			clnt->cl_protname,
			rcu_dereference(clnt->cl_xprt)->servername);
			rcu_read_unlock();
		}
	}
	rpc_force_rebind(clnt);
	/*
	 * Did our request time out due to an RPCSEC_GSS out-of-sequence
	 * event? RFC2203 requires the server to drop all such requests.
	 */
	rpcauth_invalcred(task);

retry:
	clnt->cl_stats->rpcretrans++;
	task->tk_action = call_bind;
	task->tk_status = 0;
}

/*
 * 7.	Decode the RPC reply
 */
static void
call_decode(struct rpc_task *task)
{
	struct rpc_clnt	*clnt = task->tk_client;
	struct rpc_rqst	*req = task->tk_rqstp;
	kxdrdproc_t	decode = task->tk_msg.rpc_proc->p_decode;
	__be32		*p;

	dprint_status(task);

	if (task->tk_flags & RPC_CALL_MAJORSEEN) {
		if (clnt->cl_chatty) {
			rcu_read_lock();
			printk(KERN_NOTICE "%s: server %s OK\n",
				clnt->cl_protname,
				rcu_dereference(clnt->cl_xprt)->servername);
			rcu_read_unlock();
		}
		task->tk_flags &= ~RPC_CALL_MAJORSEEN;
	}

	/*
	 * Ensure that we see all writes made by xprt_complete_rqst()
	 * before it changed req->rq_reply_bytes_recvd.
	 */
	smp_rmb();
	req->rq_rcv_buf.len = req->rq_private_buf.len;

	/* Check that the softirq receive buffer is valid */
	WARN_ON(memcmp(&req->rq_rcv_buf, &req->rq_private_buf,
				sizeof(req->rq_rcv_buf)) != 0);

	if (req->rq_rcv_buf.len < 12) {
		if (!RPC_IS_SOFT(task)) {
			task->tk_action = call_bind;
			clnt->cl_stats->rpcretrans++;
			goto out_retry;
		}
		dprintk("RPC:       %s: too small RPC reply size (%d bytes)\n",
				clnt->cl_protname, task->tk_status);
		task->tk_action = call_timeout;
		goto out_retry;
	}

	p = rpc_verify_header(task);
	if (IS_ERR(p)) {
		if (p == ERR_PTR(-EAGAIN))
			goto out_retry;
		return;
	}

	task->tk_action = rpc_exit_task;

	if (decode) {
		task->tk_status = rpcauth_unwrap_resp(task, decode, req, p,
						      task->tk_msg.rpc_resp);
	}
	dprintk("RPC: %5u call_decode result %d\n", task->tk_pid,
			task->tk_status);
	return;
out_retry:
	task->tk_status = 0;
	/* Note: rpc_verify_header() may have freed the RPC slot */
	if (task->tk_rqstp == req) {
		req->rq_reply_bytes_recvd = req->rq_rcv_buf.len = 0;
		if (task->tk_client->cl_discrtry)
			xprt_conditional_disconnect(task->tk_xprt,
					req->rq_connect_cookie);
	}
}

static __be32 *
rpc_encode_header(struct rpc_task *task)
{
	struct rpc_clnt *clnt = task->tk_client;
	struct rpc_rqst	*req = task->tk_rqstp;
	__be32		*p = req->rq_svec[0].iov_base;

	/* FIXME: check buffer size? */

	p = xprt_skip_transport_header(task->tk_xprt, p);
	*p++ = req->rq_xid;		/* XID */
	*p++ = htonl(RPC_CALL);		/* CALL */
	*p++ = htonl(RPC_VERSION);	/* RPC version */
	*p++ = htonl(clnt->cl_prog);	/* program number */
	*p++ = htonl(clnt->cl_vers);	/* program version */
	*p++ = htonl(task->tk_msg.rpc_proc->p_proc);	/* procedure */
	p = rpcauth_marshcred(task, p);
	req->rq_slen = xdr_adjust_iovec(&req->rq_svec[0], p);
	return p;
}

static __be32 *
rpc_verify_header(struct rpc_task *task)
{
	struct rpc_clnt *clnt = task->tk_client;
	struct kvec *iov = &task->tk_rqstp->rq_rcv_buf.head[0];
	int len = task->tk_rqstp->rq_rcv_buf.len >> 2;
	__be32	*p = iov->iov_base;
	u32 n;
	int error = -EACCES;

	if ((task->tk_rqstp->rq_rcv_buf.len & 3) != 0) {
		/* RFC-1014 says that the representation of XDR data must be a
		 * multiple of four bytes
		 * - if it isn't pointer subtraction in the NFS client may give
		 *   undefined results
		 */
		dprintk("RPC: %5u %s: XDR representation not a multiple of"
		       " 4 bytes: 0x%x\n", task->tk_pid, __func__,
		       task->tk_rqstp->rq_rcv_buf.len);
		goto out_eio;
	}
	if ((len -= 3) < 0)
		goto out_overflow;

	p += 1; /* skip XID */
	if ((n = ntohl(*p++)) != RPC_REPLY) {
		dprintk("RPC: %5u %s: not an RPC reply: %x\n",
			task->tk_pid, __func__, n);
		goto out_garbage;
	}

	if ((n = ntohl(*p++)) != RPC_MSG_ACCEPTED) {
		if (--len < 0)
			goto out_overflow;
		switch ((n = ntohl(*p++))) {
		case RPC_AUTH_ERROR:
			break;
		case RPC_MISMATCH:
			dprintk("RPC: %5u %s: RPC call version mismatch!\n",
				task->tk_pid, __func__);
			error = -EPROTONOSUPPORT;
			goto out_err;
		default:
			dprintk("RPC: %5u %s: RPC call rejected, "
				"unknown error: %x\n",
				task->tk_pid, __func__, n);
			goto out_eio;
		}
		if (--len < 0)
			goto out_overflow;
		switch ((n = ntohl(*p++))) {
		case RPC_AUTH_REJECTEDCRED:
		case RPC_AUTH_REJECTEDVERF:
		case RPCSEC_GSS_CREDPROBLEM:
		case RPCSEC_GSS_CTXPROBLEM:
			if (!task->tk_cred_retry)
				break;
			task->tk_cred_retry--;
			dprintk("RPC: %5u %s: retry stale creds\n",
					task->tk_pid, __func__);
			rpcauth_invalcred(task);
			/* Ensure we obtain a new XID! */
			xprt_release(task);
			task->tk_action = call_reserve;
			goto out_retry;
		case RPC_AUTH_BADCRED:
		case RPC_AUTH_BADVERF:
			/* possibly garbled cred/verf? */
			if (!task->tk_garb_retry)
				break;
			task->tk_garb_retry--;
			dprintk("RPC: %5u %s: retry garbled creds\n",
					task->tk_pid, __func__);
			task->tk_action = call_bind;
			goto out_retry;
		case RPC_AUTH_TOOWEAK:
			rcu_read_lock();
			printk(KERN_NOTICE "RPC: server %s requires stronger "
			       "authentication.\n",
			       rcu_dereference(clnt->cl_xprt)->servername);
			rcu_read_unlock();
			break;
		default:
			dprintk("RPC: %5u %s: unknown auth error: %x\n",
					task->tk_pid, __func__, n);
			error = -EIO;
		}
		dprintk("RPC: %5u %s: call rejected %d\n",
				task->tk_pid, __func__, n);
		goto out_err;
	}
	if (!(p = rpcauth_checkverf(task, p))) {
		dprintk("RPC: %5u %s: auth check failed\n",
				task->tk_pid, __func__);
		goto out_garbage;		/* bad verifier, retry */
	}
	len = p - (__be32 *)iov->iov_base - 1;
	if (len < 0)
		goto out_overflow;
	switch ((n = ntohl(*p++))) {
	case RPC_SUCCESS:
		return p;
	case RPC_PROG_UNAVAIL:
		dprintk_rcu("RPC: %5u %s: program %u is unsupported "
				"by server %s\n", task->tk_pid, __func__,
				(unsigned int)clnt->cl_prog,
				rcu_dereference(clnt->cl_xprt)->servername);
		error = -EPFNOSUPPORT;
		goto out_err;
	case RPC_PROG_MISMATCH:
		dprintk_rcu("RPC: %5u %s: program %u, version %u unsupported "
				"by server %s\n", task->tk_pid, __func__,
				(unsigned int)clnt->cl_prog,
				(unsigned int)clnt->cl_vers,
				rcu_dereference(clnt->cl_xprt)->servername);
		error = -EPROTONOSUPPORT;
		goto out_err;
	case RPC_PROC_UNAVAIL:
		dprintk_rcu("RPC: %5u %s: proc %s unsupported by program %u, "
				"version %u on server %s\n",
				task->tk_pid, __func__,
				rpc_proc_name(task),
				clnt->cl_prog, clnt->cl_vers,
				rcu_dereference(clnt->cl_xprt)->servername);
		error = -EOPNOTSUPP;
		goto out_err;
	case RPC_GARBAGE_ARGS:
		dprintk("RPC: %5u %s: server saw garbage\n",
				task->tk_pid, __func__);
		break;			/* retry */
	default:
		dprintk("RPC: %5u %s: server accept status: %x\n",
				task->tk_pid, __func__, n);
		/* Also retry */
	}

out_garbage:
	clnt->cl_stats->rpcgarbage++;
	if (task->tk_garb_retry) {
		task->tk_garb_retry--;
		dprintk("RPC: %5u %s: retrying\n",
				task->tk_pid, __func__);
		task->tk_action = call_bind;
out_retry:
		return ERR_PTR(-EAGAIN);
	}
out_eio:
	error = -EIO;
out_err:
	rpc_exit(task, error);
	dprintk("RPC: %5u %s: call failed with error %d\n", task->tk_pid,
			__func__, error);
	return ERR_PTR(error);
out_overflow:
	dprintk("RPC: %5u %s: server reply was truncated.\n", task->tk_pid,
			__func__);
	goto out_garbage;
}

static void rpcproc_encode_null(void *rqstp, struct xdr_stream *xdr, void *obj)
{
}

static int rpcproc_decode_null(void *rqstp, struct xdr_stream *xdr, void *obj)
{
	return 0;
}

static struct rpc_procinfo rpcproc_null = {
	.p_encode = rpcproc_encode_null,
	.p_decode = rpcproc_decode_null,
};

static int rpc_ping(struct rpc_clnt *clnt)
{
	struct rpc_message msg = {
		.rpc_proc = &rpcproc_null,
	};
	int err;
	msg.rpc_cred = authnull_ops.lookup_cred(NULL, NULL, 0);
	err = rpc_call_sync(clnt, &msg, RPC_TASK_SOFT | RPC_TASK_SOFTCONN);
	put_rpccred(msg.rpc_cred);
	return err;
}

struct rpc_task *rpc_call_null(struct rpc_clnt *clnt, struct rpc_cred *cred, int flags)
{
	struct rpc_message msg = {
		.rpc_proc = &rpcproc_null,
		.rpc_cred = cred,
	};
	struct rpc_task_setup task_setup_data = {
		.rpc_client = clnt,
		.rpc_message = &msg,
		.callback_ops = &rpc_default_ops,
		.flags = flags,
	};
	return rpc_run_task(&task_setup_data);
}
EXPORT_SYMBOL_GPL(rpc_call_null);

#ifdef RPC_DEBUG
static void rpc_show_header(void)
{
	printk(KERN_INFO "-pid- flgs status -client- --rqstp- "
		"-timeout ---ops--\n");
}

static void rpc_show_task(const struct rpc_clnt *clnt,
			  const struct rpc_task *task)
{
	const char *rpc_waitq = "none";

	if (RPC_IS_QUEUED(task))
		rpc_waitq = rpc_qname(task->tk_waitqueue);

	printk(KERN_INFO "%5u %04x %6d %8p %8p %8ld %8p %sv%u %s a:%ps q:%s\n",
		task->tk_pid, task->tk_flags, task->tk_status,
		clnt, task->tk_rqstp, task->tk_timeout, task->tk_ops,
		clnt->cl_protname, clnt->cl_vers, rpc_proc_name(task),
		task->tk_action, rpc_waitq);
}

void rpc_show_tasks(struct net *net)
{
	struct rpc_clnt *clnt;
	struct rpc_task *task;
	int header = 0;
	struct sunrpc_net *sn = net_generic(net, sunrpc_net_id);

	spin_lock(&sn->rpc_client_lock);
	list_for_each_entry(clnt, &sn->all_clients, cl_clients) {
		spin_lock(&clnt->cl_lock);
		list_for_each_entry(task, &clnt->cl_tasks, tk_task) {
			if (!header) {
				rpc_show_header();
				header++;
			}
			rpc_show_task(clnt, task);
		}
		spin_unlock(&clnt->cl_lock);
	}
	spin_unlock(&sn->rpc_client_lock);
}
#endif
