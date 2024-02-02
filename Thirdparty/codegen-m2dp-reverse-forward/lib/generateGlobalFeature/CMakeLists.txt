SET(globalFeature_SRCS 
cart2pol.cpp
eml_setop.cpp
findpeaks.cpp
generateGlobalFeature.cpp
generateGlobalFeature_data.cpp
generateGlobalFeature_emxAPI.cpp
generateGlobalFeature_emxutil.cpp
generateGlobalFeature_initialize.cpp
generateGlobalFeature_terminate.cpp
histcounts2.cpp
M2DP.cpp
mapElementsToBins.cpp
pca.cpp
repmat.cpp
rtGetInf.cpp
rtGetNaN.cpp
rt_nonfinite.cpp
scan2pointCloud.cpp
sort.cpp
sortIdx.cpp
svd.cpp
svd1.cpp
xaxpy.cpp
xdotc.cpp
xnrm2.cpp
xrot.cpp
xrotg.cpp
xswap.cpp
xzsvdc.cpp
) 

SET(globalFeature_HDRS
cart2pol.h
eml_setop.h
findpeaks.h
generateGlobalFeature.h
generateGlobalFeature_data.h
generateGlobalFeature_emxAPI.h
genreateGlobalFeature_emxutil.h
generateGlobalFeature_initialize.h
generateGlobalFeature_terminate.h
generateGlobalFeature_types.h
histcounts2.h
M2DP.h
mapElementsToBins.h
pca.h
repmat.h
rt_defines.h
rtGetInf.h
rtGetNaN.h
rt_nonfinite.h
scan2pointCloud.h
rtwtypes.h
sort.h
sortIdx.h
svd.h
svd1.h
xaxpy.h
xdotc.h
xnrm2.h
xrot.h
xrotg.h
xswap.h
xzsvdc.h
tmwtypes.h
) 

ADD_LIBRARY(globalFeature SHARED ${globalFeature_SRCS})


install(TARGETS globalFeature
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/globalFeature
    ARCHIVE DESTINATION lib/globalFeature)

install(FILES ${globalFeature_HDRS} DESTINATION include/globalFeature)
