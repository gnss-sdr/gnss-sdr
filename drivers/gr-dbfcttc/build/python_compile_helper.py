
import sys, py_compile
files = sys.argv[1:]
srcs, gens = files[:len(files)/2], files[len(files)/2:]
for src, gen in zip(srcs, gens):
    py_compile.compile(file=src, cfile=gen, doraise=True)
