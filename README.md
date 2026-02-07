ROS2-EKF-Fusion (Extended Kalman Filter)

ROS2-EKF-Fusion is a real-time state estimation package developed for autonomous mobile robots to address the challenges of sensor noise and odometry drift. By fusing noisy data from Wheel Odometry (Encoders) and IMU (Inertial Measurement Unit) using a custom-implemented Extended Kalman Filter (EKF) algorithm, this node generates a smooth, accurate, and drift-free robot trajectory. Built from scratch using modern C++17 and the Eigen linear algebra library without relying on black-box packages like robot_localization, this project demonstrates high-performance sensor fusion capabilities within the ROS 2 Humble framework and provides real-time visualization via Rviz2.

Quick Start:
To run the project, ensure you have ROS 2 Humble installed, clone the repository to your workspace, build it with colcon build, and run the node using ros2 run sensor_fusion_cpp fusion_node after launching a TurtleBot3 simulation.

See for working instructions: https://www.youtube.com/watch?v=Vz0V075BGaM
********************************************************************
ROS2-EKF-Fusion (Genişletilmiş Kalman Filtresi)

ROS2-EKF-Fusion, otonom mobil robotlarda sensör gürültüsü ve odometri sürüklenmesi sorunlarını çözmek için geliştirilmiş gerçek zamanlı bir durum tahmin paketidir. Tekerlek Odometrisi (Enkoderler) ve IMU (Atalet Ölçüm Birimi) verilerini, özel olarak uygulanmış bir Genişletilmiş Kalman Filtresi (EKF) algoritması kullanarak birleştiren bu düğüm; pürüzsüz, hassas ve sürüklenmeden arındırılmış bir robot yörüngesi oluşturur. robot_localization gibi hazır paketlere ihtiyaç duymadan, modern C++17 ve Eigen lineer cebir kütüphanesi kullanılarak sıfırdan geliştirilen bu proje, ROS 2 Humble çatısı altında yüksek performanslı sensör füzyonu yeteneklerini sergiler ve Rviz2 üzerinden gerçek zamanlı görselleştirme imkanı sunar.

Hızlı Başlangıç:
Projeyi çalıştırmak için ROS 2 Humble sürümünün yüklü olduğundan emin olun, depoyu çalışma alanınıza klonlayın, colcon build ile derleyin ve bir TurtleBot3 simülasyonu başlattıktan sonra ros2 run sensor_fusion_cpp fusion_node komutuyla düğümü çalıştırın.
 
Çalışma anı için bakınız: 
https://www.youtube.com/watch?v=Vz0V075BGaM
