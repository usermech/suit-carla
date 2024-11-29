# TODO 
Add to the dockerfile the following:
- Install CARLA 0.9.14 following the instructions from the [official website](https://carla.readthedocs.io/en/latest/start_quickstart/)
- Install the carla package version 0.9.14 for python.
- Install ROS2 Foxy following the instructions from the [official website](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Create a workspace from this repository and install the dependencies.

## CARLA senkronizasyonu başlatma

CARLA simülasyon ortamını başlatmak için aşağıdaki komutu çalıştırabilirsiniz:

```bash
ros2 run carla_runner carla_run --host xxx.xxx.xxx.xxx
```

Burada `xxx.xxx.xxx.xxx`, CARLA serverinin IP adresidir. Eğer `--host` parametresi belirtilmezse, varsayılan olarak `localhost` kullanılır. Bu durumda CARLA simülasyonu ve ROS2 aynı bilgisayarda çalışıyor olmalıdır.

Bu node çalıştığı zaman CARLA server zamanının kontrolünden sorumludur. Aynı zamanda trafik menajerini de başlatır. Böylece simülasyon ortamına farklı aktörler veya sensörler eklendiğinde veya çıkarıldığında simülasyon ortamı senkronize bir şekilde çalışmaya devam eder. Bu node'u diğer node'lar çalışmadan önce başlatmanız tavsiye edilir.

## Simülasyondan sensör verilerini almak

`carla_sensor_package` ROS paketi, CARLA simülasyon ortamındaki sensörlere abone olma ve yayın yapma ile ilgili işlevleri içerir. Bu paketin içerisindeki `sensor_publisher` node'u, simülasyon ortamındaki sensörlerden veri alır ve bu verileri `carla_image`, `carla_lidar` ve `carla_radar` konularından ilgili olanlara yayınlar. Bu node aktif hale geldiğinde simülasyon ortamında önceden belirlenen sensörler oluşturulur ve bu sensörlerden alınan veriler yayınlanır. Bu node çalışmıyorsa, aşağıdaki komutla node'u başlatabilirsiniz:

```bash
ros2 run carla_sensor_package sensor_publisher --host xxx.xxx.xxx.xxx
```

Sensör verilerini terminalde görmek için şu komutu çalıştırabilirsiniz:

```bash
ros2 topic echo /carla/image
ros2 topic echo /carla/lidar
ros2 topic echo /carla/radar
```

ya da bir Python kodu kullanarak bu konuya abone olabilirsiniz. Bu paketin içinde örnek bir Python kodu da bulunmaktadır. Paket içerisindeki `image_subscriber.py` dosyasını çalıştırarak sensör verilerini görselleştirebilirsiniz.

## Yaya trafiği oluşturmak

`spawn_pedestrian` isimli node ile CARLA simülasyon ortamında yaya trafiği oluşturabilirsiniz. Bu node, `carla_vehicle_spawner` paketinin bir parçasıdır. Bu node kullanılarak istenilen sayıda yaya önceden belirlenmiş bir kavşak etrafında rastgele bir şekilde oluşturulur. Oluşturulan yayalar CARLA simülasyonu tarafından kontrol edilerek otonom olarak hareket ederler. Bu node ortadan kaldırıldığında, oluşturulan yayalar da silinir. Yaya trafiği oluşturmak için aşağıdaki adımları takip edebilirsiniz:

```bash
ros2 run carla_vehicle_spawner spawn_pedestrians host:=xxx.xxx.xxx.xxx number_of_walkers:=10
```

Burada `xxx.xxx.xxx.xxx` CARLA serverinin IP adresidir ve `10` oluşturulacak yayaların sayısını belirtir.

## Bisiklet oluşturma

`spawn_bicycle` isimli node ile CARLA simülasyon ortamında bisiklet oluşturabilirsiniz. Bu node, `carla_vehicle_spawner` paketinin bir parçasıdır. Bu node kullanılarak oluşturulan bir adet bisikletli kavşak etrafında önceden belirlenmiş bir rota üzerinde hareket eder. Bu node ortadan kaldırıldığında, oluşturulan bisiklet de silinir. Bisiklet oluşturmak için aşağıdaki adımları takip edebilirsiniz:

```bash
ros2 run carla_vehicle_spawner spawn_bicycle host:=xxx.xxx.xxx.xxx
```

## Trafik ihlali yapan araç oluşturma

`spawn_road_hog` isimli node ile CARLA simülasyon ortamında trafik ihlali yapan araç oluşturabilirsiniz. Bu node, `carla_vehicle_spawner` paketinin bir parçasıdır. Bu node kullanılarak oluşturulan araçlar farklı modlarda trafik ihlali yaparlar. Eğer parametre belirtilmezse trafik ışıklarını ve işaretçilerini görmezden gelen bir araç oluşturulur. Eğer `mode` parametresi `wrong_lane` olarak belirtilirse ters yönde hareket eden bir araç oluşturulur. Eğer `mode` parametresi `speeding` olarak belirtilirse hız sınırını aşan bir araç oluşturulur. Bu node ortadan kaldırıldığında, oluşturulan araçlar da silinir. Trafik ihlali yapan araç oluşturmak için aşağıdaki adımları takip edebilirsiniz:

```bash
ros2 run carla_vehicle_spawner spawn_road_hog host:=xxx.xxx.xxx.xxx mode:=wrong_lane
```

## Trafik ışığı bilgilerini almak

CARLA simülatöründe trafik ışığı bilgilerini almak için, `carla_traffic_info` paketindeki `traffic_light_publisher` node'unu kullanmanız gerekir. Bu node, CARLA simülatöründen alınan trafik ışığı bilgilerini ROS2 `/carla/traffic_lights` konusuna yayınlar. Bu node çalışmıyorsa, aşağıdaki komutla node'u başlatabilirsiniz:

```bash
ros2 run carla_traffic_info traffic_light_publisher --host xxx.xxx.xxx.xxx
```

Burada xxx.xxx.xxx.xxx, CARLA simülatörünün IP adresidir.

Trafik ışığı bilgilerini terminalde görmek için şu komutu da çalıştırabilirsiniz:

```bash
ros2 topic echo /carla/traffic_lights
```

ya da bir Python kodu kullanarak bu konuya abone olabilirsiniz.

Trafik ışığı bilgisi, `std_msgs.msg.String` mesaj türünde bir mesaj olarak gönderilir. Mesaj şu bilgileri içerir:

```bash
data: "{'traffic_light_id': $traffic_light_id, 'traffic_light_state': $traffic_light_state, 'traffic_light_pose': $traffic_light_pose}"
```

Burada:

- traffic_light_id, CARLA simülatörü tarafından tanımlanan trafik ışığının kimliğidir.
- traffic_light_state, trafik ışığının durumudur.
- traffic_light_pose, trafik ışığının pozisyon ve yönelimini içeren bir geometry_msgs.msg.Pose mesajıdır. Sadece pozisyon ve yatay açı (yaw) önemli olduğundan, diğer yönelim değerleri sıfır olarak ayarlanmıştır.
## Acil durum aracı oluşturma

CARLA simülatöründe bir araç oluşturmak için `spawn_emergency_vehicle` nodeunu kullanmanız gerekir. Bu node, `carla_vehicle_spawner` paketinin bir parçasıdır. Bu node, CARLA simülatöründe acil durum aracı oluşturmanızı sağlar. Bu node çalışmıyorsa, aşağıdaki komutla node'u başlatabilirsiniz:

```bash
 ros2 run carla_vehicle_spawner spawn_emergency_vehicle --ros-args -p vehicle_type:='police' -p host:='xxx.xxx.xxx.xxx'
```

Bu komut bir polis aracı oluşturur. Ayrıca `vehicle_type` parametresini `firetruck` veya `ambulance` olarak değiştirerek itfaiye aracı veya ambulans oluşturabilirsiniz. Eğer parametre belirtilmezse, rastgele bir acil durum aracı oluşturulur. Eğer aşağıdakine benzer bir çıktı alırsanız, araç oluşturma işlemi başarısız olmuş demektir:

```bash
[INFO] [traffic_manager_node]: Spawn failed
```
