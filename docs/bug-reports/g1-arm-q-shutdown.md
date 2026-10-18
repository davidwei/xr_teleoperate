# G1 arms may retain teleoperation control after quitting with q

Status: open; proposed patch for review, not applied or physically validated.
Reported: 2026-10-18 (Asia/Shanghai).

## Observed behavior

The operator completed VR teleoperation and pressed `q`. The arms did not
return to the expected normal state. The operator subsequently forced the robot
into damp mode. No automatic recovery or additional robot commands were run
by the debugging agent.

The exact final arm posture, shutdown logs, measured joint positions, and
arm_sdk weight were not captured. The report therefore does not establish that
the timeout path below caused this particular physical occurrence.

## Environment and source state

- Robot: Unitree G1 EDU; configured arm controller: `G1_29`.
- Launch entry point: `bash demo.sh` in `unitree_g1_dev/pc2/vr_teleops/`.
- Runtime configuration: motion mode, BrainCo end effector, controller input
  (the preceding warnings identified `Brainco_Controller_ctrl`).
- Hardware: one installed BrainCo hand; the other hand is absent.
- XR source HEAD: `278a3dcc669eb54b0fd5ed26031e420bc7f42e88`.
- Launcher source HEAD: `599172339d532701e51d56a5d1b5f9df37df66bb`.
- Both working trees had local changes: the launcher/readiness changes and
  BrainCo single-hand DDS reader changes discussed during this session were
  uncommitted. The teleimager submodule was also dirty. These revisions are
  source anchors, not complete immutable snapshots of the physical run.
- `robot_arm.py` was unchanged from XR HEAD when inspected. The proposed
  shutdown patch in this document has not been installed.

## Reproduction

Physical repetition requires the normal robot safety setup and explicit
operator authorization. The robot was left in operator-selected damp mode;
this report does not request changing that mode or starting a test.

1. Prepare the G1 for an authorized teleoperation session in its required
   operating mode. Use the one-hand-capable BrainCo launcher and XR reader
   changes if only one hand is installed.
2. From `unitree_g1_dev/pc2/vr_teleops/`, run `bash demo.sh` with the configured
   `G1_29`, motion mode, and BrainCo end effector.
3. Complete initialization and start following XR input with `r`; operate the
   arms, then press `q` once to quit normally.
4. Capture logs from `ctrl_dual_arm_go_home start` through process exit, arm
   joint states, and the arm_sdk control-weight command if available.
5. Observe whether the arms return and control is relinquished. A failure to
   reach the zero-joint home tolerance within five seconds exercises the
   suspected defect. Do not deliberately obstruct the robot to cause a timeout.

Expected: quitting attempts the home motion, reports its outcome, and relinquishes
arm_sdk control even if homing cannot complete. Releasing teleoperation control
is distinct from confirming a normal robot operating mode or a physical pose.

Actual report: pressing `q` did not restore the expected arm state; manual damp
mode was used. Reproducibility and frequency are unknown.

## Code finding and suspected cause

`teleop/teleop_hand_and_arm.py` handles `q` by setting `STOP` and invokes
`arm_ctrl.ctrl_dual_arm_go_home()` in `finally`.

In `G1_29_ArmController.ctrl_dual_arm_go_home()` in
`teleop/robot_control/robot_arm.py`:

1. The target is set to zero for all 14 arm joints.
2. The method checks up to 100 times, sleeping 0.05 seconds between checks,
   for every measured joint angle to have magnitude below 0.05 radians.
3. The arm_sdk weight ramp from 1 to 0 is inside the successful-home branch.
4. If the approximately five-second wait expires, the method returns silently
   without executing that ramp. A state-read exception also skips it.

This is a confirmed code path. Its involvement in the observed run remains a
hypothesis because shutdown telemetry is missing. `G1_23_ArmController` has the
same structure; it was not the controller used in the reported run.

## Proposed patch for tonight's review

The patch below preserves the existing home target, tolerance, five-second
wait, and approximately two-second release ramp. It moves release into `finally`
for motion mode, logs a home timeout, and holds zero weight for 0.1 seconds so
the existing 250 Hz publisher has multiple cycles to transmit it. It covers
both standard G1 arm controllers. No-motion mode does not gain a weight ramp.

This proposal does not confirm robot receipt of zero weight, switch operating
modes, or recover a robot already in damp mode. It relies on the existing
publisher thread remaining healthy. Review the handover behavior when the arms
have not reached home before authorizing a physical trial.

```diff
--- a/teleop/robot_control/robot_arm.py
+++ b/teleop/robot_control/robot_arm.py
@@ -227,25 +227,31 @@
         return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_29_JointArmIndex])
     
     def ctrl_dual_arm_go_home(self):
-        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
+        """Attempt homing, then relinquish arm_sdk control even on timeout."""
         logger_mp.info("[G1_29_ArmController] ctrl_dual_arm_go_home start...")
-        max_attempts = 100
-        current_attempts = 0
-        with self.ctrl_lock:
-            self.q_target = np.zeros(14)
-            # self.tauff_target = np.zeros(14)
-        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
-        while current_attempts < max_attempts:
-            current_q = self.get_current_dual_arm_q()
-            if np.all(np.abs(current_q) < tolerance):
-                if self.motion_mode:
-                    for weight in np.linspace(1, 0, num=101):
-                        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight;
-                        time.sleep(0.02)
-                logger_mp.info("[G1_29_ArmController] both arms have reached the home position.")
-                break
-            current_attempts += 1
-            time.sleep(0.05)
+        try:
+            with self.ctrl_lock:
+                self.q_target = np.zeros(14)
+            tolerance = 0.05
+            for _ in range(100):
+                if np.all(np.abs(self.get_current_dual_arm_q()) < tolerance):
+                    logger_mp.info("[G1_29_ArmController] both arms have reached the home position.")
+                    break
+                time.sleep(0.05)
+            else:
+                logger_mp.warning(
+                    "[G1_29_ArmController] Arm homing timed out after 5 seconds; "
+                    "the home position was not reached."
+                )
+        finally:
+            if self.motion_mode:
+                logger_mp.info("[G1_29_ArmController] Releasing arm_sdk control...")
+                for weight in np.linspace(1, 0, num=101):
+                    self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight
+                    time.sleep(0.02)
+                # Keep zero weight active for multiple 250 Hz publish cycles.
+                time.sleep(0.1)
+                logger_mp.info("[G1_29_ArmController] arm_sdk control weight set to zero.")
 
     def set_arm_velocity_limit(self, velocity_limit = 30.0):
         '''Set the arm joint velocity limit in radians per second.'''
@@ -820,25 +826,31 @@
         return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_23_JointArmIndex])
     
     def ctrl_dual_arm_go_home(self):
-        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
+        """Attempt homing, then relinquish arm_sdk control even on timeout."""
         logger_mp.info("[G1_23_ArmController] ctrl_dual_arm_go_home start...")
-        max_attempts = 100
-        current_attempts = 0
-        with self.ctrl_lock:
-            self.q_target = np.zeros(10)
-            # self.tauff_target = np.zeros(10)
-        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
-        while current_attempts < max_attempts:
-            current_q = self.get_current_dual_arm_q()
-            if np.all(np.abs(current_q) < tolerance):
-                if self.motion_mode:
-                    for weight in np.linspace(1, 0, num=101):
-                        self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = weight;
-                        time.sleep(0.02)
-                logger_mp.info("[G1_23_ArmController] both arms have reached the home position.")
-                break
-            current_attempts += 1
-            time.sleep(0.05)
+        try:
+            with self.ctrl_lock:
+                self.q_target = np.zeros(10)
+            tolerance = 0.05
+            for _ in range(100):
+                if np.all(np.abs(self.get_current_dual_arm_q()) < tolerance):
+                    logger_mp.info("[G1_23_ArmController] both arms have reached the home position.")
+                    break
+                time.sleep(0.05)
+            else:
+                logger_mp.warning(
+                    "[G1_23_ArmController] Arm homing timed out after 5 seconds; "
+                    "the home position was not reached."
+                )
+        finally:
+            if self.motion_mode:
+                logger_mp.info("[G1_23_ArmController] Releasing arm_sdk control...")
+                for weight in np.linspace(1, 0, num=101):
+                    self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = weight
+                    time.sleep(0.02)
+                # Keep zero weight active for multiple 250 Hz publish cycles.
+                time.sleep(0.1)
+                logger_mp.info("[G1_23_ArmController] arm_sdk control weight set to zero.")
 
     def set_arm_velocity_limit(self, velocity_limit = 30.0):
         '''Set the arm joint velocity limit in radians per second.'''
```

## Offline validation of the proposal

The proposed methods were extracted using Python AST and executed with fake
joint-state readers, a fake command message, a fake clock, and no DDS imports or
robot connections. For both G1 controllers the checks passed for:

- Successful homing: the existing release ramp still executes.
- Persistent nonzero state: timeout is reported and the weight reaches zero.
- State-read exception: release executes and the original exception propagates.
- No-motion timeout: timeout is reported without changing arm_sdk weight.

The checks verified 101 monotonically decreasing weight steps from 1 to 0,
the final publication dwell, and that timeout is not logged as homing success.
Python syntax parsing passed. These checks validate control flow only; they do
not establish DDS delivery, mechanical behavior, or safety of a physical handover.

## Review and physical validation still needed

- Review and approve the proposed release-on-timeout behavior.
- Capture the original shutdown path's logs/telemetry if available to confirm
  or reject the suspected cause.
- After approval, apply the patch separately from this documentation commit.
- In an authorized supervised test, verify normal `q` shutdown, the home result,
  actual zero-weight publication, and return of control to the robot controller.
- Treat publisher failures, repeated interrupts, and process termination as
  separate unvalidated shutdown paths.
