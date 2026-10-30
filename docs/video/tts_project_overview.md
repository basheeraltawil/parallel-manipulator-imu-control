VOICE DESCRIPTION (paste into the style / instructions box):
A friendly university robotics instructor. Warm, clear, mid-pitched voice with a natural, conversational pace, about 150 words per minute. Sounds like someone who enjoys the subject and is talking to one student across a desk, not reading. Articulates numbers and symbols slowly and precisely, leaves a short breath between sentences and a clear one-second pause between paragraphs. Neutral international English. No hype, no announcer tone.

TEXT (paste everything below this line):

Welcome to Robotics Foundations. Today: the three R R R parallel manipulator.

Three identical legs, each with a motor, an elbow, and a joint on a triangular platform. Together they move the platform in a plane: left and right, forward and back, and rotation. Three motors, three degrees of freedom.

Why parallel? Every motor sits on the base, so the moving parts stay light. And the platform is held from three sides, so it is stiff and precise.

The theory starts with inverse kinematics. Each leg forms a triangle: two links of fifty millimetres, and the distance to the platform. The law of cosines gives the motor angle directly. Each elbow can bend two ways, so every pose has eight solutions.

The reverse problem is harder. The same three motor angles can hold the platform in up to six different poses.

The Jacobian connects motor speeds to platform speeds. Where its determinant reaches zero, we meet a parallel singularity. The motors are locked, yet the platform can still move. A forty degree rotation crosses one. Without protection, the platform ends twenty-seven degrees off, while the controller believes it arrived. A singularity guard stops it safely.

Then sensing. A hand-held I M U, fused with a complementary filter, turns your hand's tilt into smooth platform motion.

Where is this used? Precision alignment of wafers, displays and camera modules. Lab automation. Glue dispensing. Stabilising tools on moving bases. And teaching: kinematics, dynamics and control in one desk-sized robot.

Everything is open: the CAD, ROS two, the simulations and the firmware.

See you in the next lesson. AIBOMECH.
