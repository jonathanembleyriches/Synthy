# Synthy

Welcome to **Synthy**, a sophisticated framework for robotics research within Unreal Engine. Synthy offers unprecedented flexibility by allowing the integration of custom physics backends into your Unreal Engine simulations, facilitating advanced robotics research and development. Currently, it supports Mujoco, with plans to extend this to other physics engines.

Synthy stands out by offering utilities for handling concave collisions in Mujoco through coacd integration. This feature ensures that even complex mesh collisions are handled accurately and efficiently. Whether you are compiling everything into a single executable or setting up communications with an external engine via ZeroMQ, Synthy provides the robustness and adaptability you need. Moreover, it seamlessly integrates with ROS, allowing for sophisticated control over joint actions in your robotic simulations.

This project is crafted as an Unreal Engine plugin and promises an expanding roster of features and functionalities designed to push the boundaries of robotic simulation.

## Features

- **Custom Physics Backend**: Use Mujoco for simulation or integrate other physics engines as needed.
- **Concave Collision Handling**: Utilise coacd to manage complex mesh collisions within Mujoco.
- **Flexible Build Options**: Compile as a single executable or use ZeroMQ for communication with external engines.
- **ROS Integration**: Control joint actions and manage robotic movements within your simulation environment effectively.
- **Unreal Engine Plugin**: Easy integration with existing Unreal Engine projects, enhancing functionality without intrusive changes.

## Usage

1. **Activate the Plugin**:
   Navigate to 'Edit > Plugins' in Unreal Engine and enable the Synthy plugin.

2. **Configure Synthy**:
   Set up the desired physics backend and ROS settings via the plugin's configuration panels in Unreal Engine.

3. **Run Your Simulations**:
   With everything set up, start your robotic simulations with enhanced physics and collision management capabilities.

## Roadmap

- [x] Integrate Mujoco physics backend.
- [x] Implement coacd for concave collision support.
- [x] Enable ROS-based joint action control.
- [ ] Add support for additional physics engines.
- [ ] Expand mesh utilities for broader collision scenarios.
- [ ] Enhance documentation and user guides.
- [ ] Develop comprehensive example projects.

## Contributing


## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

Project Link: [https://github.com/yourusername/synthy](https://github.com/yourusername/synthy)

---

Thank you for considering Synthy for your robotic simulation needs. Happy simulating!
