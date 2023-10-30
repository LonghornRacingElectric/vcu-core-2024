# note this script must be executed in the root directory of the project (vcu-core-2024)
$p0 = (Get-Item .).FullName
$p1 = $p0 + "\cmake-build-debug"
cmake -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_MAKE_PROGRAM=C:/Program Files/JetBrains/CLion 2022.3.1/bin/ninja/win/ninja.exe" -G Ninja -S $p0 -B $p1
cmake --build $p1 --target vcu_core_tests -j 16
cmake --build $p1 --target vcu_core -j 16
Copy-Item "./cmake-build-debug/sil/vcu_core.exe" -Destination ".."
