function [a1, a2, a3, a4] = angles_to_corners(x,y, W, H)
%{
    This function finds the angles to the corners
     a1 = top right
     a2 = bottom right
     a3 = bottom left   
     a4 = top left
%}
    a1 = atan2(H-y,W-x);
    a2 = atan2(y,W-x);
    a3 = atan2(y,x);
    a4 = atan2(x,H-y);
end 