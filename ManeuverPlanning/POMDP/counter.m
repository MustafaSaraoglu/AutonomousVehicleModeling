function count = counter(init)
persistent currentCount;
if nargin == 1
      currentCount = init;
else
      currentCount = currentCount + 1;
end
count = currentCount;
end