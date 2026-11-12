# Reach Goal Prediction

C++ research code from my PhD in Mechanical Engineering (University of British Columbia, 2026) on modeling human reaching motion, predicting a person's reach goal from a partial trajectory, and generating human-like robot arm motion.

Author: Sara Sheikholeslami

## What the code does

**EllipseFit/** — Human reaching model and goal prediction
- Fits a best-fit plane (PCA) and a 3D ellipse to motion-capture reaching trajectories for several arm markers (upper arm, elbow, forearm, wrist, thumb, pinkie) and reports the reconstruction error.
- Predicts the reach goal from a partial trajectory: at each observed fraction of the motion, it fits the elliptical model toward each candidate goal under geometric, motion-based, and goal-consistency constraints, then scores and ranks the candidates.
- Supporting modules: plane and ellipse geometry, implicit and constrained ellipse fitting, goal clustering, and rigid transformations.

**BayesianFit/** — Bayesian model selection and maximum-likelihood fitting used to compare candidate trajectory models.

**PathPlanning/** — Robot motion generation
- Optimization-based differential inverse kinematics that retargets human arm motion onto a redundant 7-DOF robot arm (DH kinematics, Jacobians, segment-wise IK fits from the elbow, forearm and wrist).
- Export of generated motion for animation (Maya).

**GetPos/** — Helper utility used in the processing pipeline.

## Results (from the associated manuscripts)
- Goal prediction: 78–88% accuracy across four goal locations on held-out data (2,100 reaching trajectories, 25 participants).
- Elliptical model: 10,260 segmented trajectories from 38 participants; of 3,394 classified trajectories, 95.46% were elliptical, with 3.71 mm mean reconstruction error.

## Publications
- S. Sheikholeslami, H.F.M. Van der Loos, E.A. Croft. "Early Goal Prediction from Partial Human Reaching Trajectories Using a Human-Derived Elliptical Representation." IEEE Robotics and Automation Letters (RA-L), submitted.
- S. Sheikholeslami, H.F.M. Van der Loos, E.A. Croft. "Human-Derived Elliptical Trajectories Support Legible Reaching Motions Across Human and Robot Embodiments." ACM Transactions on Human-Robot Interaction (THRI), submitted.

## Notes
- Research code, shared to show the implementation. It is not packaged as a standalone library.
- Dependencies: Eigen, Ceres Solver, CMake. Some shared lab utility headers (CSV and Mathematica parsing) and the motion-capture data are not included, so the code does not build out of the box.
- No participant data is included in this repository.
- An earlier version of this work is available at github.com/personalrobotics/ReachPredictor.
