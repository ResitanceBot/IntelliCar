Integración de modelo de detección con ROS:
+ Opción A) Usar ejemplo de rosbot con el módulo de ultralytics (presuntamente v8), con esperanza de que tenga soporte retrocompatible
+ Opción B)De no servir, dado que tenemos entrenado el modelo con v7, intentar entrenar el modelo con v8 (para ello preguntar a Carlota que fichero usar para realizar el entrenamiento, y cómo indicar el dataset)
+ Opción C)Utilizar v7, pero con los módulos de yolo en local (repositorio)

Actuación en base a decisión:
+  Al detectar el bounding box a cierto tamaño, parar el vehículo