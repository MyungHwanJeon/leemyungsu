function eul = rot2rph(R)
eul = zeros(1,3);

eul(1) = atan2(R(2,1),R(1,1));
s = sin(eul(1));
c = cos(eul(1));

eul(2) = atan2(-R(3,1),R(1,1)*c + R(2,1)*s);

eul(3) = atan2(R(1,3)*s - R(2,3)*c, -R(1,2)*s + R(2,2)*c);
end