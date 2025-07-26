doxygen Doxyfile
cd docs/html/
fuser -k 8000/tcp
python3 -m http.server &
cd ../..

