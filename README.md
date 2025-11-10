# Robotics

This is the repository for group 2 of the Robotics project for the 25/26 year.

# Instrucciones actividad 1

- Escribir en la terminal el comando: robocompdsl archivo.cdsl .
- Hacer cmake .
- Hacer make

# Instrucciones Actividad 2
## Ejecutar el script de python.

Para hacerlo, ejecuta el siguiente comando:
```
python3 subcognitive.py sub.toml
```
Depende de la versión que tengas de python3 deberas usar un prefijo distinto. El archivo sub.toml guarda los comandos que se van a ejecutar.

## Abrir webots

Abrelo cómo prefieras, ejecutando webots en una terminal o buscando la aplicación a mano.

## Ejecuta el código de specificworker

Para que esta parte funcione, debes asegurarte de que has generado todos los archivos necesarios. Si no estás seguro:
1. Descarga el localiser.cdsl que se encuentra en este git.
2. Desde una terminal, vete donde has descargado el localiser.dsl, preferiblemente, en una carpeta donde solo esté este archivo para mantenerlo limpio.
3. Ejecuta:
```
robocompdsl localiser.cdsl .
```
Este comando generará varias carpetas, entre ellas, la carpeta src y generated, las cuales usaremos más adelante.
1. Copia todo el contenido de la carpeta src del github a la carpeta src de tu equipo local.
2. Copia todo el contenido de la carpeta generated del github a la carpeta generated de tu equipo local.
3. Tras esto, ejecuta el siguiente comando:
```
cmake .
```
Todos estos comandos deberás ejecutarlos desde la carpeta base del proyecto, que es la carpeta donde se encuentra el archivo localiser.cdsl. Este comando, además solo deberás ejecutarlo la primera vez que compiles el proyecto o cada vez que cambies algún cmakelists. Si lo ejecutas de normal, no está de más, aunque no es necesario.
Ejecuta ahora:
```
make
```
Esto creara el ejecutable del proyecto en la carpeta bin.

Con esto, el proyecto está listo para ser lanzado. Recordemos que, antes de lanzar el proyecto, deberás ejecutar     
```
python3 subcognitive.py sub.toml
```
Lanza ahora el proyecto, usando el siguiente comando:
```
bin/localiser etc/config
```
Si recibes algún error de tipo src/Ice, será por un problema de puertos. Deberás irte a etc/config y asegurarte que el lidar se ejecuta en el puerto 11989 y el omnirobot en el 10004
```
# Proxies for required interfaces
Proxies.Lidar3D = "lidar3d:tcp -h localhost -p 11989"
Proxies.OmniRobot = "omnirobot:tcp -h localhost -p 10004"
```


