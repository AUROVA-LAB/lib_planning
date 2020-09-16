if [ -z "$1" ]
then
    echo "Introduce un test por parametro"
else
    FILE=./config/$1
    CMAKECACHE=./config/CMakeCache.txt
    if [ -f "$FILE" ]; then
        rm ./config/$1
    fi
    if [ -f "$CMAKECACHE" ]; then
        rm ./config/CMakeCache.txt
    fi
    cmake ./config/CMakeLists.txt
    make -k -C ./config/
    if [ -f "$FILE" ]; then
        ./config/$1
    else
        echo "No existe ese test. (Escribe el nombre del test sin el .cpp)"
    fi
fi
