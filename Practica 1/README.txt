Instalación de la toolchain de ARM para la placa STM32-discovery:

	- Ejecuta el fichero de instalación "configuration.sh".  (Abre un terminal y escribe: bash configuration.sh)

	- Durante la ejecución del fichero, te podrá pedir que introduzcas la contraseña de usuario para ejecutar algunos pasos de la instalación como sudoer. También te pedira que en algunos pasos pulse la tecla "Enter" para continuar. Durante la instalación de Java saldrá una panatalla gráfica. Selecciona "OK" y pulsa "Enter" y a continuación selecciona "Yes" y pulsa "Enter"

	- Una vez finalizado la ejución del script ya están instaladas todas las herramientas necesarias para el desarrollo con la placa STM32-discovery

-------------------------------------------------------------------------------

Para descargarte un ejemplo de desarrollo con la placa STM32-discovery descargte el siguiente repositorio:

	- git clone https://github.com/rowol/stm32_discovery_arm_gcc

Se creará una carpeta llamada "stm32_discovery_arm_gcc". Entra en ella y luego en la carpeta "blinky". Esta carpeta contiene un ejemplo ya listo para compilar y descargar a la placa. Para ello deberás modificar el archivo Makefile las variables, STLINK y STM_COMMON de la siguiente forma:

	- STLINK=/home/YOUR_USER_NAME/stlink
	- STM_COMMON=/home/YOUR_USER_NAME/stm32_discovery_arm_gcc/STM32F4-DISCOVERY_FW_V1.1.0

	Nota: En lugar de YOUR_USER_NAME deberás poner tu nombre de usuario.

Para compilar el código y crear los archivos binarios necesarios ejecuta "make"
Para descargar el código binario a la placa ejecuta "make burn"

-------------------------------------------------------------------------------

Para más información, puedes consultar la página web "http://www.wolinlabs.com/blog/linux.stm32.discovery.gcc.html"

   
