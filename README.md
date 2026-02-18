# Proyecto de Robotica

(En Ingles) This is the repository for group 2 of the Robotics project for the 25/26 year.

# Instrucciones
## Ejecutar el script de python.

Para hacerlo, ejecuta el siguiente comando:
```
python3 subcognitive.py sub.toml
```
Depende de la versión que tengas de python3 deberas usar un prefijo distinto. El archivo sub.toml guarda los comandos que se van a ejecutar.

## Abrir webots

Abrelo cómo prefieras, ejecutando webots en una terminal o buscando la aplicación a mano. Además, es importante ejecutar en termimal rcnode para que todos los componentes funcionen

## Ejecuta el código de specificworker

Para que esta parte funcione, debes asegurarte de que has generado todos los archivos necesarios. Si no estás seguro:
1. Descarga el/los archivo.cdsl (chocachoca.cdsl \ localiser.cdsl) que se encuentra en este git.
2. Desde una terminal, vete donde has descargado el/los archivo.dsl, preferiblemente, en una carpeta donde solo esté este archivo para mantenerlo limpio.
3. Ejecuta para actividad 1:```robocompdsl chocachoca.cdsl .```
4. Ejecuta para actividad 2:
```
robocompdsl localiser.cdsl .
```
Este comando generará varias carpetas, pero, antes de realizar la compilación del proyecto, es importante añadir al src/CMakeList.txt los archivos que vamos a "linkear". Por lo tanto, añade estas líneas en el archivo en caso de que no estén:
```
# Sources set
LIST(APPEND SOURCES
    ../src/specificworker.cpp
    ../src/hungarian.cpp
    ../src/ransac_line_detector.cpp
    ../src/room_detector.cpp
)
# Headers set
LIST(APPEND HEADERS
    ../src/specificworker.h
    ../src/hungarian.h
    ../src/ransac_line_detector.h
    ../src/room_detector.h
)
SET(LIBS ${LIBS} tbb)
LIST(APPEND LIBS ${LIBS})
INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)
```
La carpeta src y generated, las cuales usaremos más adelante:
1. Copia todo el contenido de la carpeta src del github a la carpeta src de tu equipo local.
2. Copia todo el contenido de la carpeta generated del github a la carpeta generated de tu equipo local.
3. Tras esto, ejecuta el siguiente comando:
```
cmake .
```
Todos estos comandos deberás ejecutarlos desde la carpeta base del proyecto, que es la carpeta donde se encuentra el archivo.cdsl. Este comando, además solo deberás ejecutarlo la primera vez que compiles el proyecto o cada vez que cambies algún cmakelists. Si lo ejecutas de normal, no está de más, aunque no es necesario.
Ejecuta ahora:
```
make
```
Esto creara el ejecutable del proyecto en la carpeta bin.

Con esto, el proyecto está listo para ser lanzado. Recordemos que, antes de lanzar el proyecto, deberás ejecutar. En caso de emplear algún dispositivo externo, como
un joystick, recuerda conectarlo antes de lanzar el script:
```
python3 subcognitive.py sub.toml
```
Lanza ahora el proyecto, usando el siguiente comando:
```
bin/localiser etc/config
```
Si recibes algún error de tipo src/Ice, será por un problema de puertos. Deberás irte a etc/config y asegurarte que el lidar se ejecuta en el puerto 11989, el omnirobot en el 10004, y la cámara360 en el 10098:
```
# Proxies for required interfaces
# Proxies.Lidar3D = "lidar3d:tcp -h localhost -p 11989" BAJO
# Proxies.Lidar3D = "lidar3d:tcp -h localhost -p 11990" ALTO
Proxies.OmniRobot = "omnirobot:tcp -h localhost -p 10004"
Proxies.Camera360RGB = "camera360rgb:tcp -h localhost -p 10098"
```
Si da algún fallo de violación de segmento, añadir esta línea a generic_worker.h al principio del código:
```
#define USE_QTGUI
```


