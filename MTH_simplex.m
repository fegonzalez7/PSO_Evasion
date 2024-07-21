function T = MTH_simplex(T)
for i = 1:size(T,1)
   for j = 1:size(T,2)
      if isAlways(abs(T(i,j)) <= 1e-5,'Unknown','false') 
         T(i,j) = 0;
      end
      
   end
end