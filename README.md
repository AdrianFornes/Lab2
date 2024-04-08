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

## Conclusiones

Los controladores PI, P y PID son herramientas poderosas en el diseño de sistemas de control, proporcionando una manera efectiva de regular y estabilizar sistemas dinámicos en una variedad de aplicaciones. La elección del controlador adecuado depende de las características específicas del sistema controlado, como su respuesta dinámica, el nivel de ruido presente y la precisión requerida. Al comprender los principios y las características de cada tipo de controlador, los ingenieros pueden diseñar sistemas de control más eficientes y robustos para satisfacer las necesidades de una amplia gama de aplicaciones en robótica y automatización.
