%MATLAB2OCAMMODEL prepare ocammodel for exporting. Once the exporting is
%done, you have everything for using it with C++
%   [ocam_model] = MATLAB2OCAMMODEL(matlab_fisheye_parameter)
%   matlab_fisheye_parameter is the fisheye parameter provided after
%   calibration
%   The returned variable ocam_model is in format usable with other
%   codes provided from Scaramuzza

function [ocam_model] = matlab2ocammodel(matlab_fisheye_parameter)
intrinsics = matlab_fisheye_parameter.Intrinsics;
mapping_coeff = intrinsics.MappingCoefficients;
mapping_coeff = [mapping_coeff(1), 0.0, mapping_coeff(2), mapping_coeff(3),mapping_coeff(4)];
ocam_model.ss = mapping_coeff;
ocam_model.width = intrinsics.ImageSize(2);
ocam_model.height = intrinsics.ImageSize(1);
ocam_model.xc = intrinsics.DistortionCenter(1);
ocam_model.yc = intrinsics.DistortionCenter(2);
ocam_model.c = intrinsics.StretchMatrix(1,1);
ocam_model.d = intrinsics.StretchMatrix(1,2);
ocam_model.e = intrinsics.StretchMatrix(2,1);

ocam_model.pol = findinvpoly(ocam_model.ss, sqrt((ocam_model.width/2)^2+(ocam_model.height/2)^2));
end