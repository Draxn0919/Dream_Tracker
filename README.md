
# Dream Tracker

Dream Tracker es una aplicación de visión computacional diseñada para la detección de comportamientos relacionados con la fatiga y la somnolencia, como bostezos, microsueños y frecuencia de parpadeo. Utilizando OpenCV, Mediapipe Face Mesh y Python, el sistema monitorea en tiempo real las expresiones faciales para identificar patrones que podrían indicar que una persona está somnolienta o fatigada, lo que resulta especialmente útil en entornos de trabajo que requieren atención continua (por ejemplo, conductores u operadores de maquinaria).

![Python](https://img.shields.io/badge/Python-3.8-darkred.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5-darkblue.svg)
![Numpy](https://img.shields.io/badge/Numpy-1.19-darkgreen.svg)
![Mediapipe](https://img.shields.io/badge/Mediapipe-0.8-purple.svg)
![Pygame](https://img.shields.io/badge/Pygame-2.0.1-darkorange.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)

## Funciones principales

- Detección de Bostezos: Calcula la relación de aspecto (aspect ratio) de la boca para detectar apertura excesiva
- Detección de Microsueños: Monitorea el tiempo de cierre de los párpados, generando alertas si el tiempo de cierre es prolongado.
- Frecuencia de Parpadeo: Cuenta la tasa de parpadeo por minuto para evaluar signos de cansancio o distracción.
- Alertas en Tiempo Real: Emite alertas visuales y sonoras al detectar comportamientos críticos.
- Visualización Dinámica: Muestra el estado de detección y las métricas en tiempo real utilizando la cámara del dispositivo.


## Requerimientos

Para ejecutar el proyecto, asegúrate de tener instaladas las siguientes dependencias:

- Python 3.8+
- OpenCV (opencv-python)
- Mediapipe
- Numpy
- Pygame (para emitir alertas sonoras)

Puedes instalar todas las dependencias utilizando el siguiente comando:

```bash
pip install -r requirements.txt
```
## Instalacion

1. Clona el repositorio:
```bash
git clone https://github.com/Draxn0919/Dream_Tracker.git
cd Dream_Tracker
```
2. Ejecuta el script principal:
```bash
python main.py
```
3. Conecta tu cámara o usa un video de prueba desde la carpeta test_videos/.

4. Observa los resultados en la ventana de OpenCV que mostrará en tiempo real los puntos de referencia faciales y las alertas correspondientes.
## Explicación Técnica
El proyecto utiliza el modelo Face Mesh de Mediapipe para rastrear los 468 puntos de referencia del rostro. Se extraen los puntos claves de la boca y los ojos para calcular las siguientes métricas:

- Relación de Aspecto del Ojo (EAR): Se utiliza para determinar el tiempo de cierre de los ojos. Si el valor cae por debajo de un umbral predefinido durante un tiempo específico, se marca como microsueño.
- Relación de Aspecto de la Boca (MAR): Se emplea para identificar la apertura de la boca y detectar si una persona está bostezando.
- Frecuencia de Parpadeo: El sistema cuenta el número de parpadeos en un intervalo de tiempo, indicando la fatiga si la tasa de parpadeo es anormal.
## Ecuaciones Clave

Para la detección de fatiga, el proyecto utiliza métricas basadas en relaciones de aspecto de ojos y boca, definidas por las siguientes ecuaciones:

### 1. Eye Aspect Ratio (EAR)

El **EAR** se utiliza para detectar el cierre de los ojos mediante la relación de distancia entre puntos de referencia verticales y horizontales en el ojo:


EAR =  ( ||p_2 - p_6|| + ||p_3 - p_5|| ) / ( 2 *||p_1 - p_4||)


Donde:

- \( p_1, p_2, p_3, p_4, p_5, p_6 \) son los puntos de referencia del ojo generados por Face Mesh.

### 2. Mouth Aspect Ratio (MAR)

El **MAR** mide la apertura de la boca usando las distancias verticales y horizontales entre puntos clave de los labios:


MAR = ( ||p_2 - p_8|| ) / ( ||p_1 - p_4|| )


Donde:

- ( p_1, p_2, p_4, p_8 ) son los puntos de referencia de la boca generados por Face Mesh.

### 3. Definición de Microsueño

Un **Microsueño** se detecta si el **EAR** cae por debajo de un umbral específico (\< 0.20) durante un período continuo de tiempo (> 2 segundos).

Estas ecuaciones permiten identificar cambios sutiles en las expresiones faciales que son indicativos de fatiga o somnolencia.

## Resultados
Al ejecutar el proyecto con la cámara en tiempo real o utilizando videos de prueba, el sistema debería ser capaz de:

- Identificar bostezos con alta precisión.
- Detectar microsueños al calcular el tiempo de cierre de los párpados.
- Analizar la frecuencia de parpadeo para evaluar el estado de atención de la persona.
- Emitir alertas visuales y auditivas ante comportamientos de fatiga.
## Mejora y Futuras Implementaciones
- Integración de redes neuronales para mejorar la precisión de la detección.
- Soporte para múltiples personas en una misma secuencia de video.
- Optimización de la velocidad para reducir la latencia en la detección en tiempo real.
- Panel de control para visualizar las estadísticas de detección y métricas a largo plazo.


## Contributing

Si deseas contribuir a este proyecto, sigue estos pasos:

- Haz un fork del repositorio.
- Crea una nueva rama (git checkout -b feature/nueva-funcionalidad).
- Realiza tus cambios y realiza un commit (git commit -m 'Añadir nueva funcionalidad').
- Envía tus cambios a la rama principal (git push origin feature/nueva-funcionalidad).
- Abre un Pull Request para revisar tus cambios.

## LICENSE

Este proyecto está licenciado bajo la **Licencia MIT**. Consulta el archivo [LICENSE](./LICENSE) para más detalles.




## Authors

- Github: [@Draxn0919](https://github.com/Draxn0919)
- Email: danrivera505@gmail.com
- linkedin: [Daniel Rivera]([https://www.linkedin.com/in/daniel-rivera-yepes])
