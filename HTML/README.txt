1. place the HTML files you wish to upload to your MCU inside the fs directory
2. Inside windows explorer window type cmd in the folder path
3. run perl makefsdata.pl
4. place fsdata.c and fsdata_custom.c inside Middlewares/Third_Party/LwIP/apps/httpd
5. dont forget to exclude the files from the build process