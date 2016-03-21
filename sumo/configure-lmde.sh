if [ -e "/usr/lib/libgdal.so" ]; then
	echo "libgdal found. good.\n "
else
	libgdal=`ls -x /usr/lib/libgdal*.so | cut -d" " -f1`
	if [ $libgdal -eq "" ]; then
		echo "gdal lib not found, aborting"
		exit
	fi

	sudo ln -s $libgdal /usr/lib/libgdal.so		
fi

export CXXFLAGS="-lGL"
./configure --with-proj-gdal --with-proj-libraries=/usr/lib --with-gdal-libraries=/usr/lib --with-gdal-includes=/usr/include/gdal --enable-traci --enable-internal-lanes --enable-debug
