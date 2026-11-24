<launch>

    <!-- ローバー制御 -->
    <include file="$(find lightrover_ros)/launch/roadmap_drive.launch" />

    <!-- YDLIDAR -->
    <include file="$(find lightrover_ros)/launch/ydlidar_only.launch" />

</launch>
