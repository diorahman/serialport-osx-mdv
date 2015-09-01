serial: test.c
	clang test.c -o serial -framework CoreFoundation -framework IOKit
test: serial
	./serial
clean:
	rm -fr serial
