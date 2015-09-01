mdv-test: app.c mdv.c
	clang app.c mdv.c -o mdv-test -framework CoreFoundation -framework IOKit
test: mdv-test
	./mdv-test
clean:
	rm -fr mdv-test
