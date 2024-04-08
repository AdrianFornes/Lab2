# Lab2
Report of second laboratory

## Resumen
Este informe detalla las actividades realizadas en el laboratorio 2, que abarca desde prácticas básicas hasta avanzadas utilizando ROS (Robot Operating System). En las prácticas básicas, se creó un paquete ROS llamado Practicas_lab, se implementaron nodos de publicación y suscripción, y se ejecutaron para demostrar la comunicación entre nodos. En las prácticas avanzadas, se exploraron conceptos como el control de robots simulados en turtlesim, desarrollando un controlador por teclado, dibujando formas geométricas y comparando diferentes estrategias de control como el PID.

## Introducción

En el campo de la robótica y el control de sistemas dinámicos, los controladores Proporcional (P), Proporcional-Integral (PI) y Proporcional-Integral-Derivativo (PID) son herramientas fundamentales para lograr un comportamiento deseado en los sistemas controlados. Estos controladores son ampliamente utilizados en una variedad de aplicaciones debido a su capacidad para regular y estabilizar sistemas en tiempo real.

## Controladores PI, P y PID

- **Controlador Proporcional (P)**: El controlador proporcional es el más simple de los tres, y su acción se basa únicamente en el error presente entre la salida deseada y la salida real del sistema. El controlador P ajusta la señal de control proporcionalmente al error, lo que significa que cuanto mayor sea el error, mayor será la corrección aplicada. Sin embargo, este controlador puede resultar en oscilaciones alrededor del punto de ajuste deseado y no garantiza la estabilidad del sistema.

- **Controlador Proporcional-Integral (PI)**: El controlador PI agrega una componente integral al controlador proporcional. Esta componente integral tiene en cuenta el historial completo de los errores pasados y ajusta la señal de control para eliminar el error acumulado. El controlador PI es capaz de eliminar el error en estado estacionario y mejorar la estabilidad del sistema, pero puede provocar una respuesta lenta a cambios repentinos en la referencia.

- **Controlador Proporcional-Integral-Derivativo (PID)**: El controlador PID combina las acciones proporcional, integral y derivativa para proporcionar un control más completo y preciso del sistema. La componente derivativa anticipa la tendencia del error y proporciona una respuesta rápida a cambios en la referencia. Esto ayuda a mejorar la estabilidad y la respuesta dinámica del sistema, reduciendo el tiempo de respuesta y minimizando el sobrepico. El controlador PID es el más utilizado en aplicaciones industriales debido a su capacidad para controlar una amplia gama de sistemas con diferentes características dinámicas.

## Problemas

- **Basic**
- Crear un paquete llamado Practicas_lab de ros con dependencias rospy, roscpp y std_msgs
- Colocar los archivos listener.py y talker.py
- Compilar el paquete.
- Ejecutar el talker
- Ejecutar el listener
- Concluir sobre su funcionamiento.
  
- **Medium**
- Crear un control por teclado para turtlesim
- Dibujar un cuadrado y un triángulo equilátero con turtlesim (Sin controlador)
  
- **Advanced**
- Control de posición para turtlesim (P)
- Control de posición para turtlesim (PI)
- Control de posición para turtlesim (PID)

## Soluciones  

- Para la reproduccion del listerner y el talker.py se modificaron los archivos launch para poder ejecutar los archivos con la terminal en Ros, tambien para esto se ejecutaron simultaniamente los dos archivos para poder tener un buen funcionamiento y se pudiera escuchar y hablar.
  
- Para el nivel medio se realizo un codigo en el cual se ejecutaban movimientos para que la tortuga se moviera de acuerdo lo escrito en el codigo y girar mediante los angulos dados y repitiendolo varias veces para poder realizar el triangulo y el cubo, se realizan de manera aleatoria las lineas debido a que no hay controlador y lo unico que conoce el simulador es su posicion inicial y los movimientos del codigo. Para comtrolar la tortuga se hizo un codigo en el cual podiamos mover la tortuga en diferentes direcciones con el teclado sabiendo la posicion de la tortuga y indicando la accion de cada tecla como lo es ir hacia adelante, atras, izquierda y derecha.

- Para los controladores se hicieron tres codigos los cuales se pide una coordenada en x, otra coordenada en y y el angulo en el que se requiere que la tortuga se mueva, haciendo yn recorrido a la posicion deseada con el angulo requerido asi para los diferentes controladores P, PI y PID.

**Los codigos se encuentran en la carpeta de lab2 que esta añadida en el documento.**

    
## Conclusiones

Los fundamentos de ROS se estudiaron en este laboratorio, así como las prácticas básicas y avanzadas de control de sistemas robóticos simulados. Todos los ejercicios requeridos se realizaron correctamente, lo que permitió reconocer y comprender los diferentes tipos de controladores, incluidos los controladores Proporcional (P), Proporcional-Integral (PI) y Proporcional-Integral-Derivativo (PID).

La ejecución exitosa de los ejercicios demostró que el equipo tenía la capacidad de aplicar la teoría del control a la práctica en un entorno de simulación robótica. La comprensión de los principios de los controladores y su aplicación en ROS establece una base sólida para futuros proyectos y desarrollos en el campo de la robótica y la automatización.

Este laboratorio brindó una oportunidad importante para explorar las capacidades de ROS y mejorar las habilidades en el diseño, desarrollo y control de sistemas robóticos. Los estudiantes están mejor preparados para enfrentar desafíos más complejos en el futuro al desarrollar soluciones robóticas innovadoras y eficientes.

