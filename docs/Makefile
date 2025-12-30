.PHONY: FORCE clean

out/foc.pdf: FORCE | out
	latexrun foc.tex -o $@ -O build

out:
	mkdir -p out

clean:
	-rm -rd build
