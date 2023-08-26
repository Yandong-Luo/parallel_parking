# parallel_parking
R Gator Parallel Parking

1. **Perception:** Reimplemented the PointPillars algorithm on real vehicles to detect object by pytorch, getting the goal position of parallel parking. Generated the grid map as the configuration space for planning. For this part, you can find more details from here: [Reimplement-PointPillar](https://github.com/Yandong-Luo/Reimplement-PointPillar)
2. **Planning:** Generated the path of parallel parking by hybrid A*. Added vehicle steering constraints, penalties for turning, steering, reversing. Combined the reeds shepp path and A* algorithm without obstacle as the heuristic part. Smoothed the path by the cost of curvature, smoothness, and obstacle.
   
#### Result

##### Simulation TEST

[![KITTI ROS TEST](https://res.cloudinary.com/marcomontalbano/image/upload/v1683487276/video_to_markdown/images/youtube--HrnisTXsBnI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/otTrGEX5Vzc)



