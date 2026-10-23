"""The task model and its Temporal contract, shared by every side of a run.

REST validates against it, WorkflowGateway dispatches with it, and the worker
(RobotWorkflow, RobotActivities) runs it. It imports from none of them, so
none of them has to import another to agree on a step, a signal or a queue.
"""
