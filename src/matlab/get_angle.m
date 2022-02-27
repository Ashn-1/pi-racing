function angle = get_angle(p1, p2, p3)

v1 = p2 - p1;
v2 = p3 - p2;

angle = dot(v1, v2) / (sqrt(dot(v1, v1)) * sqrt(dot(v2, v2)));
if abs(angle) > 1
    angle = angle / abs(angle);
end
angle = acos(angle);

end

