// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

use kernel::{
    list::{AtomicListArcTracker, List, ListArc, TryNewListArc},
    prelude::*,
    sync::lock::{spinlock::SpinLockBackend, Guard},
    sync::{Arc, LockedBy},
};

use crate::{
    defs::*,
    process::{NodeRefInfo, Process, ProcessInner},
    thread::Thread,
    BinderReturnWriter, DArc, DLArc, DeliverToRead,
};

mod wrapper;
pub(crate) use self::wrapper::CritIncrWrapper;

#[derive(Debug)]
pub(crate) struct CouldNotDeliverCriticalIncrement;

/// Keeps track of how this node is scheduled.
///
/// There are two ways to schedule a node to a work list. Just schedule the node itself, or
/// allocate a wrapper that references the node and schedule the wrapper. These wrappers exists to
/// make it possible to "move" a node from one list to another - when `do_work` is called directly
/// on the `Node`, then it's a no-op if there's also a pending wrapper.
///
/// Wrappers are generally only needed for zero-to-one refcount increments, and there are two cases
/// of this: weak increments and strong increments. We call such increments "critical" because it
/// is critical that they are delivered to the thread doing the increment. Some examples:
///
/// * One thread makes a zero-to-one strong increment, and another thread makes a zero-to-one weak
///   increment. Delivering the node to the thread doing the weak increment is wrong, since the
///   thread doing the strong increment may have ended a long time ago when the command is actually
///   processed by userspace.
///
/// * We have a weak reference and are about to drop it on one thread. But then another thread does
///   a zero-to-one strong increment. If the strong increment gets sent to the thread that was
///   about to drop the weak reference, then the strong increment could be processed after the
///   other thread has already exited, which would be too late.
///
/// Note that trying to create a `ListArc` to the node can succeed even if `has_normal_push` is
/// set. This is because another thread might just have popped the node from a todo list, but not
/// yet called `do_work`. However, if `has_normal_push` is false, then creating a `ListArc` should
/// always succeed.
///
/// Like the other fields in `NodeInner`, the delivery state is protected by the process lock.
struct DeliveryState {
    /// Is the `Node` currently scheduled?
    has_pushed_node: bool,

    /// Is a wrapper currently scheduled?
    ///
    /// The wrapper is used only for strong zero2one increments.
    has_pushed_wrapper: bool,

    /// Is the currently scheduled `Node` scheduled due to a weak zero2one increment?
    ///
    /// Weak zero2one operations are always scheduled using the `Node`.
    has_weak_zero2one: bool,

    /// Is the currently scheduled wrapper/`Node` scheduled due to a strong zero2one increment?
    ///
    /// If `has_pushed_wrapper` is set, then the strong zero2one increment was scheduled using the
    /// wrapper. Otherwise, `has_pushed_node` must be set and it was scheduled using the `Node`.
    has_strong_zero2one: bool,
}

impl DeliveryState {
    fn should_normal_push(&self) -> bool {
        !self.has_pushed_node && !self.has_pushed_wrapper
    }

    fn did_normal_push(&mut self) {
        assert!(self.should_normal_push());
        self.has_pushed_node = true;
    }

    fn should_push_weak_zero2one(&self) -> bool {
        !self.has_weak_zero2one && !self.has_strong_zero2one
    }

    fn can_push_weak_zero2one_normally(&self) -> bool {
        !self.has_pushed_node
    }

    fn did_push_weak_zero2one(&mut self) {
        assert!(self.should_push_weak_zero2one());
        assert!(self.can_push_weak_zero2one_normally());
        self.has_pushed_node = true;
        self.has_weak_zero2one = true;
    }

    fn should_push_strong_zero2one(&self) -> bool {
        !self.has_strong_zero2one
    }

    fn can_push_strong_zero2one_normally(&self) -> bool {
        !self.has_pushed_node
    }

    fn did_push_strong_zero2one(&mut self) {
        assert!(self.should_push_strong_zero2one());
        assert!(self.can_push_strong_zero2one_normally());
        self.has_pushed_node = true;
        self.has_strong_zero2one = true;
    }

    fn did_push_strong_zero2one_wrapper(&mut self) {
        assert!(self.should_push_strong_zero2one());
        assert!(!self.can_push_strong_zero2one_normally());
        self.has_pushed_wrapper = true;
        self.has_strong_zero2one = true;
    }
}

struct CountState {
    /// The reference count.
    count: usize,
    /// Whether the process that owns this node thinks that we hold a refcount on it. (Note that
    /// even if count is greater than one, we only increment it once in the owning process.)
    has_count: bool,
}

impl CountState {
    fn new() -> Self {
        Self {
            count: 0,
            has_count: false,
        }
    }
}

struct NodeInner {
    /// Strong refcounts held on this node by `NodeRef` objects.
    strong: CountState,
    /// Weak refcounts held on this node by `NodeRef` objects.
    weak: CountState,
    delivery_state: DeliveryState,
    /// The number of active BR_INCREFS or BR_ACQUIRE operations. (should be maximum two)
    ///
    /// If this is non-zero, then we postpone any BR_RELEASE or BR_DECREFS notifications until the
    /// active operations have ended. This avoids the situation an increment and decrement get
    /// reordered from userspace's perspective.
    active_inc_refs: u8,
    /// List of `NodeRefInfo` objects that reference this node.
    refs: List<NodeRefInfo, { NodeRefInfo::LIST_NODE }>,
}

#[pin_data]
pub(crate) struct Node {
    ptr: u64,
    cookie: u64,
    pub(crate) flags: u32,
    pub(crate) owner: Arc<Process>,
    inner: LockedBy<NodeInner, ProcessInner>,
    #[pin]
    links_track: AtomicListArcTracker,
}

kernel::list::impl_list_arc_safe! {
    impl ListArcSafe<0> for Node {
        tracked_by links_track: AtomicListArcTracker;
    }
}

impl Node {
    pub(crate) fn new(
        ptr: u64,
        cookie: u64,
        flags: u32,
        owner: Arc<Process>,
    ) -> impl PinInit<Self> {
        pin_init!(Self {
            inner: LockedBy::new(
                &owner.inner,
                NodeInner {
                    strong: CountState::new(),
                    weak: CountState::new(),
                    delivery_state: DeliveryState {
                        has_pushed_node: false,
                        has_pushed_wrapper: false,
                        has_weak_zero2one: false,
                        has_strong_zero2one: false,
                    },
                    active_inc_refs: 0,
                    refs: List::new(),
                },
            ),
            ptr,
            cookie,
            flags,
            owner,
            links_track <- AtomicListArcTracker::new(),
        })
    }

    /// Insert the `NodeRef` into this `refs` list.
    ///
    /// # Safety
    ///
    /// It must be the case that `info.node_ref.node` is this node.
    pub(crate) unsafe fn insert_node_info(
        &self,
        info: ListArc<NodeRefInfo, { NodeRefInfo::LIST_NODE }>,
    ) {
        self.inner
            .access_mut(&mut self.owner.inner.lock())
            .refs
            .push_front(info);
    }

    /// Insert the `NodeRef` into this `refs` list.
    ///
    /// # Safety
    ///
    /// It must be the case that `info.node_ref.node` is this node.
    pub(crate) unsafe fn remove_node_info(
        &self,
        info: &NodeRefInfo,
    ) -> Option<ListArc<NodeRefInfo, { NodeRefInfo::LIST_NODE }>> {
        // SAFETY: We always insert `NodeRefInfo` objects into the `refs` list of the node that it
        // references in `info.node_ref.node`. That is this node, so `info` cannot possibly be in
        // the `refs` list of another node.
        unsafe {
            self.inner
                .access_mut(&mut self.owner.inner.lock())
                .refs
                .remove(info)
        }
    }

    /// An id that is unique across all binder nodes on the system. Used as the key in the
    /// `by_node` map.
    pub(crate) fn global_id(&self) -> usize {
        self as *const Node as usize
    }

    pub(crate) fn get_id(&self) -> (u64, u64) {
        (self.ptr, self.cookie)
    }

    pub(crate) fn inc_ref_done_locked(
        self: &DArc<Node>,
        _strong: bool,
        owner_inner: &mut ProcessInner,
    ) -> Option<DLArc<Node>> {
        let inner = self.inner.access_mut(owner_inner);
        if inner.active_inc_refs == 0 {
            pr_err!("inc_ref_done called when no active inc_refs");
            return None;
        }

        inner.active_inc_refs -= 1;
        if inner.active_inc_refs == 0 {
            // Having active inc_refs can inhibit dropping of ref-counts. Calculate whether we
            // would send a refcount decrement, and if so, tell the caller to schedule us.
            let strong = inner.strong.count > 0;
            let has_strong = inner.strong.has_count;
            let weak = strong || inner.weak.count > 0;
            let has_weak = inner.weak.has_count;

            let should_drop_weak = !weak && has_weak;
            let should_drop_strong = !strong && has_strong;

            // If we want to drop the ref-count again, tell the caller to schedule a work node for
            // that.
            let need_push = should_drop_weak || should_drop_strong;

            if need_push && inner.delivery_state.should_normal_push() {
                let list_arc = ListArc::try_from_arc(self.clone()).ok().unwrap();
                inner.delivery_state.did_normal_push();
                Some(list_arc)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub(crate) fn update_refcount_locked(
        self: &DArc<Node>,
        inc: bool,
        strong: bool,
        count: usize,
        owner_inner: &mut ProcessInner,
    ) -> Option<DLArc<Node>> {
        let is_dead = owner_inner.is_dead;
        let inner = self.inner.access_mut(owner_inner);

        // Get a reference to the state we'll update.
        let state = if strong {
            &mut inner.strong
        } else {
            &mut inner.weak
        };

        // Update the count and determine whether we need to push work.
        let need_push = if inc {
            state.count += count;
            // TODO: This method shouldn't be used for zero-to-one increments.
            !is_dead && !state.has_count
        } else {
            if state.count < count {
                pr_err!("Failure: refcount underflow!");
                return None;
            }
            state.count -= count;
            !is_dead && state.count == 0 && state.has_count
        };

        if need_push && inner.delivery_state.should_normal_push() {
            let list_arc = ListArc::try_from_arc(self.clone()).ok().unwrap();
            inner.delivery_state.did_normal_push();
            Some(list_arc)
        } else {
            None
        }
    }

    pub(crate) fn incr_refcount_allow_zero2one(
        self: &DArc<Self>,
        strong: bool,
        owner_inner: &mut ProcessInner,
    ) -> Result<Option<DLArc<Node>>, CouldNotDeliverCriticalIncrement> {
        let is_dead = owner_inner.is_dead;
        let inner = self.inner.access_mut(owner_inner);

        // Get a reference to the state we'll update.
        let state = if strong {
            &mut inner.strong
        } else {
            &mut inner.weak
        };

        // Update the count and determine whether we need to push work.
        state.count += 1;
        if is_dead || state.has_count {
            return Ok(None);
        }

        // Userspace needs to be notified of this.
        if !strong && inner.delivery_state.should_push_weak_zero2one() {
            assert!(inner.delivery_state.can_push_weak_zero2one_normally());
            let list_arc = ListArc::try_from_arc(self.clone()).ok().unwrap();
            inner.delivery_state.did_push_weak_zero2one();
            Ok(Some(list_arc))
        } else if strong && inner.delivery_state.should_push_strong_zero2one() {
            if inner.delivery_state.can_push_strong_zero2one_normally() {
                let list_arc = ListArc::try_from_arc(self.clone()).ok().unwrap();
                inner.delivery_state.did_push_strong_zero2one();
                Ok(Some(list_arc))
            } else {
                state.count -= 1;
                Err(CouldNotDeliverCriticalIncrement)
            }
        } else {
            // Work is already pushed, and we don't need to push again.
            Ok(None)
        }
    }

    pub(crate) fn incr_refcount_allow_zero2one_with_wrapper(
        self: &DArc<Self>,
        strong: bool,
        wrapper: CritIncrWrapper,
        owner_inner: &mut ProcessInner,
    ) -> Option<DLArc<dyn DeliverToRead>> {
        match self.incr_refcount_allow_zero2one(strong, owner_inner) {
            Ok(Some(node)) => Some(node as _),
            Ok(None) => None,
            Err(CouldNotDeliverCriticalIncrement) => {
                assert!(strong);
                let inner = self.inner.access_mut(owner_inner);
                inner.strong.count += 1;
                inner.delivery_state.did_push_strong_zero2one_wrapper();
                Some(wrapper.init(self.clone()))
            }
        }
    }

    pub(crate) fn update_refcount(self: &DArc<Self>, inc: bool, count: usize, strong: bool) {
        self.owner
            .inner
            .lock()
            .update_node_refcount(self, inc, strong, count, None);
    }

    pub(crate) fn populate_counts(
        &self,
        out: &mut BinderNodeInfoForRef,
        guard: &Guard<'_, ProcessInner, SpinLockBackend>,
    ) {
        let inner = self.inner.access(guard);
        out.strong_count = inner.strong.count as _;
        out.weak_count = inner.weak.count as _;
    }

    pub(crate) fn populate_debug_info(
        &self,
        out: &mut BinderNodeDebugInfo,
        guard: &Guard<'_, ProcessInner, SpinLockBackend>,
    ) {
        out.ptr = self.ptr as _;
        out.cookie = self.cookie as _;
        let inner = self.inner.access(guard);
        if inner.strong.has_count {
            out.has_strong_ref = 1;
        }
        if inner.weak.has_count {
            out.has_weak_ref = 1;
        }
    }

    pub(crate) fn force_has_count(&self, guard: &mut Guard<'_, ProcessInner, SpinLockBackend>) {
        let inner = self.inner.access_mut(guard);
        inner.strong.has_count = true;
        inner.weak.has_count = true;
    }

    fn write(&self, writer: &mut BinderReturnWriter, code: u32) -> Result {
        writer.write_code(code)?;
        writer.write_payload(&self.ptr)?;
        writer.write_payload(&self.cookie)?;
        Ok(())
    }

    /// This is split into a separate function since it's called by both `Node::do_work` and
    /// `NodeWrapper::do_work`.
    fn do_work_locked(
        &self,
        writer: &mut BinderReturnWriter,
        mut guard: Guard<'_, ProcessInner, SpinLockBackend>,
    ) -> Result<bool> {
        let inner = self.inner.access_mut(&mut guard);
        let strong = inner.strong.count > 0;
        let has_strong = inner.strong.has_count;
        let weak = strong || inner.weak.count > 0;
        let has_weak = inner.weak.has_count;

        if weak && !has_weak {
            inner.weak.has_count = true;
            inner.active_inc_refs += 1;
        }

        if strong && !has_strong {
            inner.strong.has_count = true;
            inner.active_inc_refs += 1;
        }

        let no_active_inc_refs = inner.active_inc_refs == 0;
        let should_drop_weak = no_active_inc_refs && (!weak && has_weak);
        let should_drop_strong = no_active_inc_refs && (!strong && has_strong);
        if should_drop_weak {
            inner.weak.has_count = false;
        }
        if should_drop_strong {
            inner.strong.has_count = false;
        }
        if no_active_inc_refs && !weak {
            // Remove the node if there are no references to it.
            guard.remove_node(self.ptr);
        }
        drop(guard);

        if weak && !has_weak {
            self.write(writer, BR_INCREFS)?;
        }
        if strong && !has_strong {
            self.write(writer, BR_ACQUIRE)?;
        }
        if should_drop_strong {
            self.write(writer, BR_RELEASE)?;
        }
        if should_drop_weak {
            self.write(writer, BR_DECREFS)?;
        }

        Ok(true)
    }
}

impl DeliverToRead for Node {
    fn do_work(
        self: DArc<Self>,
        _thread: &Thread,
        writer: &mut BinderReturnWriter,
    ) -> Result<bool> {
        let mut owner_inner = self.owner.inner.lock();
        let inner = self.inner.access_mut(&mut owner_inner);

        assert!(inner.delivery_state.has_pushed_node);
        if inner.delivery_state.has_pushed_wrapper {
            // If the wrapper is scheduled, then we are either a normal push or weak zero2one
            // increment, and the wrapper is a strong zero2one increment, so the wrapper always
            // takes precedence over us.
            assert!(inner.delivery_state.has_strong_zero2one);
            inner.delivery_state.has_pushed_node = false;
            inner.delivery_state.has_weak_zero2one = false;
            return Ok(true);
        }

        inner.delivery_state.has_pushed_node = false;
        inner.delivery_state.has_weak_zero2one = false;
        inner.delivery_state.has_strong_zero2one = false;

        self.do_work_locked(writer, owner_inner)
    }

    fn cancel(self: DArc<Self>) {}

    fn should_sync_wakeup(&self) -> bool {
        false
    }
}

/// Represents something that holds one or more ref-counts to a `Node`.
///
/// Whenever process A holds a refcount to a node owned by a different process B, then process A
/// will store a `NodeRef` that refers to the `Node` in process B. When process A releases the
/// refcount, we destroy the NodeRef, which decrements the ref-count in process A.
///
/// This type is also used for some other cases. For example, a transaction allocation holds a
/// refcount on the target node, and this is implemented by storing a `NodeRef` in the allocation
/// so that the destructor of the allocation will drop a refcount of the `Node`.
pub(crate) struct NodeRef {
    pub(crate) node: DArc<Node>,
    /// How many times does this NodeRef hold a refcount on the Node?
    strong_node_count: usize,
    weak_node_count: usize,
    /// How many times does userspace hold a refcount on this NodeRef?
    strong_count: usize,
    weak_count: usize,
}

impl NodeRef {
    pub(crate) fn new(node: DArc<Node>, strong_count: usize, weak_count: usize) -> Self {
        Self {
            node,
            strong_node_count: strong_count,
            weak_node_count: weak_count,
            strong_count,
            weak_count,
        }
    }

    pub(crate) fn absorb(&mut self, mut other: Self) {
        assert!(
            Arc::ptr_eq(&self.node, &other.node),
            "absorb called with differing nodes"
        );
        self.strong_node_count += other.strong_node_count;
        self.weak_node_count += other.weak_node_count;
        self.strong_count += other.strong_count;
        self.weak_count += other.weak_count;
        other.strong_count = 0;
        other.weak_count = 0;
        other.strong_node_count = 0;
        other.weak_node_count = 0;
    }

    pub(crate) fn clone(&self, strong: bool) -> Result<NodeRef> {
        if strong && self.strong_count == 0 {
            return Err(EINVAL);
        }
        Ok(self
            .node
            .owner
            .inner
            .lock()
            .new_node_ref(self.node.clone(), strong, None))
    }

    /// Updates (increments or decrements) the number of references held against the node. If the
    /// count being updated transitions from 0 to 1 or from 1 to 0, the node is notified by having
    /// its `update_refcount` function called.
    ///
    /// Returns whether `self` should be removed (when both counts are zero).
    pub(crate) fn update(&mut self, inc: bool, strong: bool) -> bool {
        if strong && self.strong_count == 0 {
            return false;
        }
        let (count, node_count, other_count) = if strong {
            (
                &mut self.strong_count,
                &mut self.strong_node_count,
                self.weak_count,
            )
        } else {
            (
                &mut self.weak_count,
                &mut self.weak_node_count,
                self.strong_count,
            )
        };
        if inc {
            if *count == 0 {
                *node_count = 1;
                self.node.update_refcount(true, 1, strong);
            }
            *count += 1;
        } else {
            *count -= 1;
            if *count == 0 {
                self.node.update_refcount(false, *node_count, strong);
                *node_count = 0;
                return other_count == 0;
            }
        }
        false
    }
}

impl Drop for NodeRef {
    // This destructor is called conditionally from `Allocation::drop`. That branch is often
    // mispredicted. Inlining this method call reduces the cost of those branch mispredictions.
    #[inline(always)]
    fn drop(&mut self) {
        if self.strong_node_count > 0 {
            self.node
                .update_refcount(false, self.strong_node_count, true);
        }
        if self.weak_node_count > 0 {
            self.node
                .update_refcount(false, self.weak_node_count, false);
        }
    }
}
