# android15-6.6-partner_predev branch

This branch is dedicated for the small subset of features for android16, that
can't be developed on android-mainline.

Every patch merged in this branch **must**:

* be reviewed (and +2ed) by a person from this OWNERS file;
* have a bug associated with it via `Bug: <bug_id>` in the commit message
    * the associated bug should be filed in the
      [pkvm component](https://b.corp.google.com/issues?q=componentid:956175)
    * and must provide a clear explanation why the patch should
      be merged in this branch.

If you have more questions, please reach out to android-kvm@google.com
