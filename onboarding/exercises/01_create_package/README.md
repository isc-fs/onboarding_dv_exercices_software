# Exercise 01 — Create a ROS 2 package

Goal: understand what a **package** is and how a **node** lives inside one.
You build a tiny package from a skeleton, then run a "hello" node.

This exercise stays in `onboarding/` only — nothing in `pipeline/` yet.

## What you will produce

A package named `hello_onboarding` that:

- Declares itself to ROS (`package.xml`, ament resource marker)
- Installs a console script `hello` → your node entry point
- When run, logs `Hello from <your_name>` once per second

## Skeleton layout (already started)

```
01_create_package/
  README.md                 ← you are here
  hello_onboarding/         ← Python module (the code ROS imports)
    __init__.py
    hello_node.py           ← STUDENT TODOs
  package.xml               ← STUDENT TODOs
  setup.py                  ← STUDENT TODOs
  setup.cfg
  resource/hello_onboarding ← empty marker file (do not delete)
```

## Steps

1. Open `package.xml` and fill every `# STUDENT` field (name, description, deps).
2. Open `setup.py` and wire the `console_scripts` entry point to
   `hello_onboarding.hello_node:main`.
3. Open `hello_node.py` and complete the `HelloNode` class (see TODOs).
4. Build and run from a ROS 2 sourced shell (or the pipeline Docker image):

```bash
# From a colcon workspace that contains this folder, e.g. after:
#   ln -s $(pwd)/onboarding/exercises/01_create_package <ws>/src/hello_onboarding
# or copy the package into src/
cd <ws>
colcon build --packages-select hello_onboarding
source install/setup.bash
ros2 run hello_onboarding hello
```

5. Confirm you see a repeating log line. Stop with Ctrl+C.

## Concepts to take away

| Piece | Role |
|-------|------|
| **Package** | Sharable unit of ROS software (msgs, nodes, launch, libs). One folder + `package.xml`. |
| **Node** | A running process that uses the ROS client library (`rclpy` / `rclcpp`) to talk on the graph. |
| **Entry point** | How `ros2 run <pkg> <exec>` finds your `main()`. |
| **`package.xml`** | Declares name, license, and dependencies so the build system and other packages can find you. |
| **`setup.py` / `CMakeLists.txt`** | Tells the build how to install Python or C++ artifacts. |

Next: theory in the GUIDE, then publishers/subscribers.
