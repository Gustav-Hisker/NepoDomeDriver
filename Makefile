install:
	cd build; \
	cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug ../; \
	make; \
	sudo make install

server: install
	sudo indiserver -v nepo_dome