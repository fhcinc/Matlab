function c = bitcount(v)

count = 0;
for i = 1:28
    if( bitand( v, 1) ~= 0)
        count = count + 1;
    end
    v = bitshift(v, -1);
    %disp(v);
end

c = count;