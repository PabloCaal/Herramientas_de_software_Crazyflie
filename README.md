# Herramientas de software para el control individual y seguro del cuadricópetro Crazyflie 2.1
_Este repositorio contiene el desarrollo de herramientas de software y documentación relacionada con el proyecto de control individual y seguro del cuadricóptero Crazyflie 2.1 utilizando la placa de expansión de posicionamiento Flow Deck. El objetivo es facilitar el uso independiente de este microdron en entornos educativos y de investigación en la Universidad del Valle de Guatemala. Este repositorio centraliza todos los recursos necesarios para realizar experimentos de control y navegación con el dron Crazyflie, permitiendo a estudiantes e investigadores utilizar y mejorar este sistema de manera accesible y segura._

<p align="center">
  <img src="multimedia/github_crazyflie_con_marker.jpg" width="27%" alt="Imagen 1">
  <img src="multimedia/github_crazyflie_con_flowdeck.jpg" width="25.45%" alt="Imagen 2">
  <img src="multimedia/github_crazyflie_atravesando_obstaculo.png" width="32.85%" alt="Imagen 3">
</p>

## Descripción
Este proyecto implementa un conjunto de herramientas de software para controlar el dron Crazyflie 2.1 de forma individual, utilizando la placa de expansión Flow Deck para mejorar su precisión de posicionamiento. A lo largo del desarrollo, se probaron y validaron varios algoritmos de control básico en Python, compatibles con Matlab, para permitir la integración entre ambos entornos. Además, se llevaron a cabo experimentos en el ecosistema de investigación Robotat, adaptando el dron a las condiciones y requisitos específicos de las prácticas de laboratorio y de investigación en la Universidad del Valle de Guatemala​.

## Índice
- [Descripción](#descripción)
- [Estructura del proyecto](#estructura-del-repositorio)
- [Requisitos previos](#requisitos-previos)
- [Instalación](#instalación)
- [Uso](#uso)
  - [Ejemplo de ejecución en Python](#ejemplo-de-ejecución-en-python)
  - [Ejemplo de ejecución en Matlab](#ejemplo-de-ejecución-en-matlab)
- [Ejemplos](#ejemplos)
- [Documentación](#documentación)
- [Licencia](#referencias)
- [Referencias](#agradecimientos)

## Estructura del repositorio
- **experimentos**: Conjunto de scripts en Matlab de los experimentos realizados para validar a las herramientas de software desarrolladas .
- **laboratorios**: Conjunto de scripts de los experimentos de los laboratorios, junto a las guías de laboratorio elaboradas.
- **multimedia**: Imágenes y videos que ilustran el proyecto en acción.
- **paquete_de_herramientas**: Herramientas de software y scripts necesarios para controlar al dron Crazyflie.

## Requisitos previos
Para este proyecto, se utilizaron Python y Matlab como herramientas principales para el desarrollo y control del dron Crazyflie:

- **Python 3.11.0**: Usado para desarrollar las funciones de control básico del Crazyflie, utilizando las siguientes librerías:
  - `cflib`: Librería oficial de Crazyflie para comunicación y control del dron.
  - `time`: Para la gestión de temporización.
  - `sys`: Para la manipulación de parámetros del sistema.
  - `event`: Para el manejo de eventos necesarios en el control.
  - `logging`: Para registrar información relevante sobre el funcionamiento del dron y depuración de errores.

- **Matlab '24.1.0.2603908 (R2024a) Update 3'**: Usado como centro de control para ejecutar experimentos más complejos con el dron Crazyflie. Matlab permite invocar funciones de Python mediante comandos específicos, proporcionando una integración eficaz entre ambos lenguajes. Esta versión de Matlab es compatible con Python 3.11.0, según la matriz de compatibilidad oficial de Matlab, asegurando una comunicación fluida y estable entre Matlab y Python.

## Instalación
1. **Instalar Python**: Asegúrate de tener Python 3.11.0 instalado. Puedes verificar tu versión de Python con el siguiente comando:
   ```bash
   python --version
Asegúrate de agregar Python al PATH durante la instalación. 

2. **Instalar cflib**: Esta librería es fundamental para la comunicación con el dron Crazyflie. Para instalarla, ejecuta el siguiente comando:
   ```bash
   pip install cflib
Este comando instalará las dependencias necesarias para el funcionamiento del dron, como pyusb, libusb-package, scipy, numpy, entre otras.

3. **Configurar Python en Matlab**: Si deseas ejecutar funciones de Python desde Matlab, asegúrate de que Matlab reconozca la versión correcta de Python. Para comprobar esto, abre Matlab y ejecuta:
   ```bash
   pyenv
   
4. Si la versión de Python no es la correcta, cambia la configuración ejecutando el siguiente comando, reemplazando 'ruta/a/python' con la ruta a tu instalación de Python:
   ```bash
   pyenv('Version', 'ruta/a/python')
Esto permitirá que Matlab utilice la instalación de Python correcta y acceda a las librerías de Crazyflie.
   
## Uso y ejemplos


## Documentación

## Referencias
- Bitcraze, documentación de Crazyflie y sus módulos.
- Universidad del Valle de Guatemala, guías de laboratorio y experimentos desarrollados.

## Agradecimientos
* Asesor: Msc. Miguel Zea
---
Por [PabloCaal](https://github.com/PabloCaal)
