$p = (Get-Item .).FullName + "\cmake-build-debug"
cmake --build $p --target vcu_core_tests -j 16
cmake --build $p --target vcu_core -j 16
Copy-Item "./cmake-build-debug/sil/vcu_core.exe" -Destination ".."
