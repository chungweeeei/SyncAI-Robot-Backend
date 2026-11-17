# TODO

只列項目；動某一項就開 branch，過程寫在該項目底下，做完整段刪掉。

## 進行中

### 1. landmark detect 接受 ambiguous，朝向改用視線方向
branch: fix/landmark-ambiguous-heading
改動點: syncai_backend/gateways/landmark/landmark.py, syncai_backend/interfaces/rest/routers/landmark.py, syncai_backend/helpers/landmark_heading.py, syncai_backend/helpers/planar_pose.py, syncai_backend/gateways/failure.py, syncai_backend/main.py

#### 影響範圍
每格寫三件事：現在怎麼做｜檔:行｜動或不動。沒有關係寫「無｜不動」。不得留空、寫同上或問號。

| 實例＼層 | ↕ exceptions | ↕ helpers | ↓ database | ↕ tasks | ↕ repositories | ↕ gateways | ↕ services | ↕ subscribers | ↕ temporal | ↕ interfaces | ↕ 未列入模組表:syncai_backend | 本身 | ↑ main | ↑ 未列入模組表:(根目錄) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| syncai_backend/gateways/landmark/landmark.py（改動點） | 無｜不動 | 呼叫 landmark_heading.sighting_in_map 組出 LandmarkSighting｜syncai_backend/helpers/planar_pose.py:18｜動 | 無｜不動 | 無｜不動 | 無｜不動 | poll 經 ros.call_service，失敗用 failure.fail 標記｜syncai_backend/gateways/landmark/landmark.py:86｜動 | 無｜不動 | 改由注入的共用 TF buffer 在影像時間查 map←相機｜syncai_backend/subscribers/tf.py:23｜動 | 無｜不動 | router 讀 detect() 的回傳｜syncai_backend/interfaces/rest/routers/landmark.py:91｜動 | 無｜不動 | 以 map 為 target_frame poll，收 consecutive≥3 且非 ambiguous 的單筆｜syncai_backend/gateways/landmark/landmark.py:60｜動 | init_landmark_gateway(logger, node)｜syncai_backend/main.py:129｜動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/landmark.py（改動點） | ConflictError / BadRequestError / UpstreamError 對應 409/400/502｜syncai_backend/interfaces/rest/routers/landmark.py:8｜不動 | planar_yaw 算 tag 朝向，改成呼叫 landmark_heading｜syncai_backend/interfaces/rest/routers/landmark.py:104｜動 | 無｜不動 | 無｜不動 | robot_repo 取地面 z、catalog 取載入的地圖｜syncai_backend/interfaces/rest/routers/landmark.py:65｜不動 | landmark_gw.detect 與 failure_code 對應錯誤，多一個 off_axis｜syncai_backend/interfaces/rest/routers/landmark.py:96｜動 | 無｜不動 | 無｜不動 | 無｜不動 | server.py 用 landmark_gw、robot_repo、map_catalog_repo 建 router｜syncai_backend/interfaces/rest/server.py:301｜不動 | 無｜不動 | 把單筆偵測轉成 x,y,z,theta 回應｜syncai_backend/interfaces/rest/routers/landmark.py:125｜動 | 經 RestDeps 注入｜syncai_backend/main.py:268｜不動 | 無｜不動 |
| syncai_backend/helpers/landmark_heading.py（改動點） | 無｜不動 | 呼叫 planar_yaw_of_rotation 算 tag 在 map 的朝向｜syncai_backend/helpers/planar_pose.py:18｜動 | 無｜不動 | 無｜不動 | 無｜不動 | gateway 產出 LandmarkSighting｜syncai_backend/gateways/landmark/landmark.py:60｜動 | 無｜不動 | 無｜不動 | 無｜不動 | router 呼叫朝向判斷｜syncai_backend/interfaces/rest/routers/landmark.py:104｜動 | 無｜不動 | 新檔：LandmarkSighting、相機座標系轉 map 的 sighting_in_map、視線朝向與 θ 的純函式｜無｜動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/planar_pose.py（改動點） | 無｜不動 | quat_to_rotation_matrix｜syncai_backend/helpers/planar_pose.py:11｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | gateway 不直接用，經 landmark_heading 間接用｜無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | router 目前直接用 planar_yaw｜syncai_backend/interfaces/rest/routers/landmark.py:11｜不動 | 無｜不動 | 6-DOF 位姿投影到地面的 yaw；拆出吃旋轉矩陣的 planar_yaw_of_rotation，planar_yaw 改呼叫它｜syncai_backend/helpers/planar_pose.py:18｜動 | 無｜不動 | 無｜不動 |
| syncai_backend/gateways/failure.py（改動點） | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/tasks/config.py:109｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:143｜不動 | LandmarkGateway 用 fail(Failure.LANDMARK_*) 標記｜syncai_backend/gateways/landmark/landmark.py:112｜動 | 既有引用，與這次改動無關｜syncai_backend/services/map_switch.py:115｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:15｜不動 | router 用 failure_code 對應 409/400｜syncai_backend/interfaces/rest/routers/landmark.py:96｜動 | 無｜不動 | Failure 列舉有 LANDMARK_NOT_DETECTED / LANDMARK_UNKNOWN，新增 LANDMARK_OFF_AXIS｜syncai_backend/gateways/failure.py:68｜動 | 無｜不動 | 無｜不動 |
| syncai_backend/main.py（改動點） | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:14｜不動 | 既有引用，與這次改動無關｜syncai_backend/tasks/config.py:10｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:23｜不動 | init_landmark_gateway 多傳 tf_buffer｜syncai_backend/main.py:129｜動 | 既有引用，與這次改動無關｜syncai_backend/main.py:47｜不動 | init_tf_listener 提前到 gateway 之前建立｜syncai_backend/main.py:209｜動 | 既有引用，與這次改動無關｜syncai_backend/main.py:15｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:21｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:13｜不動 | 組裝順序：repos → gateways → services → TF listener → subscribers｜syncai_backend/main.py:123｜動 | 無｜不動 | 既有引用，與這次改動無關｜setup.py:61｜不動 |
| syncai_backend/exceptions.py | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/helpers/move_guard.py:15｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/tasks/schema.py:7｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:15｜不動 | 既有引用，與這次改動無關｜syncai_backend/gateways/failure.py:25｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/map_import.py:25｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:12｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/landmark.py:8｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/gateways/ros.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/disk.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:16｜不動 | 既有引用，與這次改動無關｜syncai_backend/gateways/recording/recording.py:13｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/floors.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:17｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/maps/floors.py:11｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/keepout.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/map_archive.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/map_import.py:26｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/maps/archive.py:14｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/move_guard.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/map_usage.py:29｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:367｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:679｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/occupancy_grid.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/pcd_to_gridmap.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/pgm.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:18｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/pointcloud.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/subscribers/map_cloud_subscriber.py:14｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/maps/cloud.py:12｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:28｜不動 | 無｜不動 |
| syncai_backend/helpers/route_preview.py | 既有引用，與這次改動無關｜syncai_backend/exceptions.py:42｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/tasks/catalogue.py:79｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:737｜不動 | 既有引用，與這次改動無關｜syncai_backend/gateways/failure.py:41｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/controller_schema.py:217｜不動 | 既有引用，與這次改動無關｜syncai_backend/subscribers/path_subscriber.py:9｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:365｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/maps/archive.py:29｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/single_flight.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/network.py:11｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/system_config.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/repositories/map/catalog.py:19｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/map_switch.py:45｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/helpers/traversable.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/map.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/map.py:19｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/map.py:18｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/map.py:23｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:34｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:24｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/network.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/network.py:9｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/network.py:11｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/network.py:10｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/pointcloud.py | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/helpers/pcd_to_gridmap.py:39｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/pointcloud.py:6｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/subscribers/map_cloud_subscriber.py:14｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:28｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/recording.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/recording.py:9｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/recording.py:16｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/recording.py:10｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:31｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/robot.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/robot.py:12｜不動 | 既有引用，與這次改動無關｜syncai_backend/helpers/system_config.py:80｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/robot.py:15｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/robot.py:13｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/robot.py:16｜不動 | 既有引用，與這次改動無關｜syncai_backend/subscribers/robot_state_subscriber.py:4｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:27｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:23｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/schedule.py | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/schedule.py:8｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/schedule.py:19｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/schedule.py:13｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/settings.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/settings.py:6｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/task.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:16｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:41｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:46｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:18｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:44｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:39｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task.py:47｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:36｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:25｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/task_template.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task_template.py:38｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task_template.py:37｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task_template.py:39｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task_template.py:57｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/task_template.py:51｜不動 | 既有引用，與這次改動無關｜syncai_backend/services/map_usage.py:31｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:37｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:25｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/telemetry.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/telemetry.py:10｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/subscribers/local_nav_subscriber.py:17｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:30｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/teleop.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/teleop.py:6｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 無｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/tts.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/tts.py:6｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/tasks/schema.py:98｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/tts.py:7｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/temporal/activities.py:32｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:42｜不動 | 無｜不動 |
| syncai_backend/interfaces/rest/routers/webrtc.py | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/webrtc.py:6｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 既有引用，與這次改動無關｜syncai_backend/interfaces/rest/routers/webrtc.py:7｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:43｜不動 | 無｜不動 |
| syncai_backend/logger.py | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 無｜不動 | 與 landmark 無關｜無｜不動 | 既有引用，與這次改動無關｜syncai_backend/main.py:13｜不動 | 無｜不動 |

#### 快速 catchup

操作員把狗擺在充電樁前 1.0–1.3 m 按 Detect，`POST /api/v1/landmarks/detect` 一律 409：detector 在這個距離每一幀都是 `ambiguous=true`（兩個 PnP 解分不出來），backend 只收 `ambiguous=false`。兩解的位置相同、朝向對「tag 指向相機的視線方向」左右對稱，所以改成：位置照收，ambiguous 時朝向用視線方向，量 θ（tag 朝向與視線方向的夾角，就是狗偏離充電樁正前方的角度），θ 超過 10° 拒收。決定（2026-11-17 使用者確認）：相機位置由 backend 用共用 TF buffer 自己轉、門檻 10°、超門檻回 409 `landmark_off_axis`、規劃寫在這份 TODO。

#### 診斷

- 病因：gateway 假設 6 s 內兩解的歧義會自己消失，但歧義由幾何決定（detector 只在誤差差 3 倍 + 0.3 px 或相機離 tag < 0.6 m 時才清），狗站著不動永遠不清；backend 的朝向只有 detector 選的那個解一個來源，影像分不出時沒有第二個來源，也量不到誤差，只能整個拒收。
- 同類：backend 呼叫 `detect_landmark` 1 處、讀 `ambiguous` 1 處、`planar_yaw` 使用者 1 處；backend 自己查 TF 的 3 處都是 subscriber；Workspace `feature/charging` 的 `DetectLandmark` / `PublishLandmark` 也丟 ambiguous（另一個 repo，記在下方待辦）。
- 牽動：前端 `detectCharger` 把 theta 存成 CHARGE vertex → charge.xml 沿 theta 推 1.3 m 當 staging pose；回應只增欄位、新 409 代碼前端會照樣顯示句子；文件 `docs/rest-api.md`、`docs/ros-interfaces.md`、`CLAUDE.md`。
- 已驗證（robot01）：兩解對視線方向對稱（10:09 ±10.2°、13:46 ±0.85°）；`target_frame=""` 回應帶 `robot01/camera_optical_frame` 與影像時間；在影像時間查 map←相機 < 1 ms，自己轉出的 tag 與 detector 直接回的 map 差 ≤ 1 mm、yaw 相同。
- 未驗證：對稱只在視線接近水平時成立（tag 與相機高度相近）；10° 門檻下 staging 偏 23 cm 時最後一公尺修不修得回來；13:39 那 3 幀 −164° 雜訊幀的原因；暗處曝光階梯走完時 6 s 內收不收得滿 5 筆。

#### 設計

- 動局部：只改 backend 的 landmark 路徑，分層不動。
- gateway 問相機座標系（`target_frame=""`），用注入的共用 TF buffer 在影像時間查 map←相機，每筆轉成純資料 `LandmarkSighting`（tag 位置、tag 朝向、相機位置、ambiguous、consecutive、age、誤差）。
- 純函式 `helpers/landmark_heading.py`：`sighting_in_map` 把相機座標系的偵測加上 TF 轉成 map 的 `LandmarkSighting`（朝向用 `planar_pose` 拆出的 `planar_yaw_of_rotation`，不另寫一份）；位置取中位數；5 筆都不 ambiguous 用 detector 的朝向（誤差最小那筆），否則用視線方向；θ 取重投影誤差最小那一筆的（2026-11-17 實測 detector 的位姿在兩組擬合之間跳：0.23 px 那組 θ≈8°、0.95 px 那組 θ≈17°，中位數會在 200 與 409 之間跳，使用者選誤差最小那筆）。
- router：ambiguous 且 θ > 10° 回 409 `landmark_off_axis`；回應加 `ambiguous`、`off_axis_deg`。
- 份數：把偵測轉成 waypoint 位姿 1 → 1（router 內 → 純函式）；轉到 map 1 → 1（detector → backend）；判斷穩定 1 → 1（gateway）。

#### 單位（依序）

1. gateway 改問相機座標系、backend 自己轉到 map，回傳 `LandmarkSighting`；收件規則不變；`main.py` 提前建立 TF listener 並注入。驗證：landmark 的 gateway / router 測試（假的 TF buffer、TF 失敗 502）、整套 pytest、ruff；機上用一次性 container 跑 branch 的 gateway，與舊路徑比 ≤ 1 mm、≤ 0.1°。
2. 純函式 `landmark_heading` 與測試。驗證：unit test（正對、偏左、偏右、全不 ambiguous、混合、跨 ±180°、一筆雜訊幀）與 stress test（10⁵ 組隨機幾何：朝向誤差 ≤ θ、θ 超門檻必拒收）。
3. 接受 ambiguous、收滿 5 筆、超門檻拒收、回應加欄位、每種拒收都有 warning log、更新文件。驗證：gateway / router 測試；機上現在站位 200、橫移 30 cm 409。

#### 目前做到哪

單位 1–3 的程式與測試完成（整套 1582 passed），合成一個 commit（使用者要求先 commit）；機上：一次性 container 跑真的 router 3 次皆 200、theta 相差 < 0.04°；backend container 已用這個 branch 重 build，部署後 API 200。剩：前端實測；文件（`docs/rest-api.md` 的 detect 路由與 CHARGE vertex theta、`docs/ros-interfaces.md` 的 LandmarkGateway、`CLAUDE.md` 的分層說明、TF listener 的說明）使用者要求先停，尚未更新。

## 待辦

- backend 分層是依技術（routers / services / gateways / helpers），不是 feature 底下再分 domain / application / adapter。
- Workspace `charge.xml` 的 `DetectLandmark` / `PublishLandmark` 在 1.3 m 的 staging pose 丟掉 ambiguous，CHARGE step 會卡在那裡。
- Interface `DetectLandmark.srv` 的註解寫「backend 不呼叫它」，已過期。
- Workspace `CLAUDE.md` 的「backend 用到的 ROS 介面」表沒有 `detect_landmark`。
- Frontend 認得 `landmark_off_axis`，並顯示 θ。
