class BehaviorTreeError(Exception):
    pass


class BehaviorTreeRunner:
    CONTROL_NODES = {"Sequence", "Fallback", "Retry"}

    def __init__(self, trees, primitive_runner):
        self.trees = trees or {}
        self.primitive_runner = primitive_runner

    def task_ids(self):
        return set(self.trees.keys())

    def run(self, task_id, goal_handle):
        if task_id not in self.trees:
            raise BehaviorTreeError(f"Unknown behavior tree task: {task_id}")
        self._run_node(self.trees[task_id], goal_handle)

    def flatten_types(self, task_id):
        if task_id not in self.trees:
            raise BehaviorTreeError(f"Unknown behavior tree task: {task_id}")
        result = []
        self._collect_types(self.trees[task_id], result)
        return result

    def _collect_types(self, node, result):
        if not isinstance(node, dict):
            raise BehaviorTreeError("Behavior tree node must be a mapping")
        node_type = node.get("type")
        if not node_type:
            raise BehaviorTreeError("Behavior tree node is missing type")
        result.append(node_type)
        for child in node.get("children", []):
            self._collect_types(child, result)
        child = node.get("child")
        if child is not None:
            self._collect_types(child, result)

    def _run_node(self, node, goal_handle):
        if not isinstance(node, dict):
            raise BehaviorTreeError("Behavior tree node must be a mapping")
        node_type = node.get("type")
        if not node_type:
            raise BehaviorTreeError("Behavior tree node is missing type")

        if node_type == "Sequence":
            children = self._children(node)
            for child in children:
                self._run_node(child, goal_handle)
            return

        if node_type == "Fallback":
            children = self._children(node)
            last_error = None
            for child in children:
                try:
                    self._run_node(child, goal_handle)
                    return
                except Exception as exc:
                    if self._is_nonrecoverable(exc):
                        raise
                    last_error = exc
            if last_error is not None:
                raise last_error
            raise BehaviorTreeError("Fallback requires at least one child")

        if node_type == "Retry":
            attempts = int(node.get("attempts", 1))
            if attempts <= 0:
                raise BehaviorTreeError("Retry attempts must be > 0")
            child = node.get("child")
            if child is None:
                children = self._children(node)
                if len(children) != 1:
                    raise BehaviorTreeError("Retry requires exactly one child")
                child = children[0]
            last_error = None
            for _attempt in range(attempts):
                try:
                    self._run_node(child, goal_handle)
                    return
                except Exception as exc:
                    if self._is_nonrecoverable(exc):
                        raise
                    last_error = exc
            if last_error is not None:
                raise last_error
            return

        self.primitive_runner(node_type, node, goal_handle)

    def _children(self, node):
        children = node.get("children", [])
        if not isinstance(children, list):
            raise BehaviorTreeError(f"{node.get('type')} children must be a list")
        return children

    @staticmethod
    def _is_nonrecoverable(exc):
        return exc.__class__.__name__ == "MissionCanceled"
