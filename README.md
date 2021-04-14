# IIC2685 2021-1

### Lab 1

topicos:

- `/yocs_cmd_vel_mux/input/navigation`: mensajes de velocidad
- `/odom`: leer odometria
- `/goal_list`
- `/occupancy_state`
- `/camera/depth/image_raw`

nodos:

- `dead_reckoning_nav`
- `obstacle_detector`

## actividad 1: avanzar y rotar

`dead_reckoning_nav`:

topic: `/goal_list` (subscribe)

- `apply_velocity(lin_vel, ang_vel, time)`
- `move_robot_to_destination(goal_pose)`
- `move_action_cb`

diagram: `/goal_list` -> `move_action_cb(goal_pose)` -> `move_robot_to_destination(goal_pose)` -> `while not in goal_pose: apply_velocity()`

- 3 vueltas cuadradas (1m2), registrar coordenadas en tiempo real (/real_pose) + coordenadas registradas por odometria (/odom)
- analizar diferencias
- `move_and_rotate.launch`

## actividad 2: percepcion basica

`obstacle_detector`:

topics:

- `/occupancy_state` (publish): std_msgs/String (obstacle_left,...,free) < 50 cm
- `/camera/depth/image_raw` (subscribe): depth image

diagram: `/camera/depth/image_raw` -> process image, detect obstacles -> `/occupancy_state`

- agregar y eliminar paredes mostrando deteccion en terminal
- `obstacle_detector.launch`

## actividad 3: uniendo accion y percepcion

- `/dead_reckoning_nav` debe subscribirse a `/occupancy_state` y avanzar solo si el mensaje es "free"
- agregar paquete `sound_play`. El robot debe hablar cuando encuentra un obstaculo
- `action_and_perception.launch` -> lanzar todo
