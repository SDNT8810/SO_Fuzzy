function T = Choose_Experience(TD,Long_Memory_Experience)
    T(Long_Memory_Experience.Window_Size) = 0;
    T(1:length(TD)) = TD;
end