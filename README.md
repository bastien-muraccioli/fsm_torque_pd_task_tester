# FSMTorquePDTaskTester

A demonstration **mc_rtc FSM controller** showcasing the use of the `TorquePDJointTask`.

This controller illustrates how to perform **joint-level torque control with PD feedback** using the mc_rtc task framework. The implementation builds upon existing mc_rtc torque tasks and adds configurable feedforward torques and compensation mechanisms.

---

## Overview

`FSMTorquePDTaskTester` demonstrates the use of the **TorquePDJointTask**, which computes joint torques using a PD control law:

τ = Kp (q_d - q) + Kd (q̇_d - q̇)

Where:

* q — current joint positions
* q̇ — current joint velocities
* q_d — desired joint positions
* q̇_d — desired joint velocities
* Kp — stiffness gain
* Kd — damping gain

The task is built on top of the **mc_rtc `TorqueTask`**, which already provides:

* **Coriolis compensation**
* **Gravity compensation**
* **External torque compensation**

An **additional feedforward torque vector** can optionally be added to the final torque command.

---

## Default Behavior

Unless specified otherwise, the controller uses the following defaults:

* Desired joint positions initialized to the **current configuration** q_d = q
  

* Desired joint velocities initialized to **zero** q̇_d = 0

* Derivative gains automatically set as: Kd = 2*sqrt{Kp}

This choice provides **critical damping behavior** for many systems.

---

## Repository Branches

### `main`

Targets **Kinova Gen3** torque control.

### `h1`

Example implementation for the **Unitree H1 humanoid robot**.

---

## Dependencies

### Core dependencies

* `mc_rtc`
* `mc_kinova`
* `mc_h1`
* `mc_residual_estimation` *(used for external force compensation)*

⚠️ The **recommended installation method** is via:

**mc_rtc_superbuild**

This simplifies installing all dependencies and building the controller.

---

## Simulation

Simulation support is provided through **MuJoCo**.

Required packages:

* `mc_mujoco`
* `mj_kinova_description`
* `mj_h1_description`

---

## Real Robot Deployment

### Kinova Gen3

Required package:

* `mc_kortex`

### Unitree H1

Required package:

* `mc_unitree2`

---

## Purpose

This controller is intended as:

* A **minimal example** of torque PD control with mc_rtc
* A **testing framework** for torque-based tasks
* A **reference implementation** for users implementing torque controllers in mc_rtc