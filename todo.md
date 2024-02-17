Integración de modelo de detección con ROS:
+ Opción A) Usar ejemplo de rosbot con el módulo de ultralytics (presuntamente v8), con esperanza de que tenga soporte retrocompatible -> NO VIABLE
+ Opción B)De no servir, dado que tenemos entrenado el modelo con v7, intentar entrenar el modelo con v8 (para ello preguntar a Carlota que fichero usar para realizar el entrenamiento, y cómo indicar el dataset)
+ Opción C)Utilizar v7, pero con los módulos de yolo en local (repositorio)

Actuación en base a decisión:
+  Al detectar el bounding box a cierto tamaño, parar el vehículo

-------------------------
Things To Fix:
+ Lidar frame y ego_vehicle frame no cuadran a veces al ejecutar. Lidar frame sale sobre map frame. Podria compararse la posicion de los frames ego_vehicle y lidar y si no son muy parecidas es por fallo y en ese caso se haria transformada para llevar lidar frame a ego_vehicle. 