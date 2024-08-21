file(REMOVE_RECURSE
  "libvolk_gnsssdr.a"
  "libvolk_gnsssdr.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/volk_gnsssdr_static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
