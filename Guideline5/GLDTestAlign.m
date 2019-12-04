
close all;
%
merfiles = { 'C:\Users\cserb\ownCloud\SVN\PJ388\Design Output\Software\Guideline5000\Guideline5000\bin\x64\Release\Waveforms\Patient 111\Procedure 1\Pass 1\01-Nov-2019-13-36-09HMS1.gld'};
eogfiles = { 'C:\Users\cserb\ownCloud\SVN\PJ388\Design Output\Software\Guideline5000\Guideline5000\bin\x64\Release\Waveforms\Patient 111\Procedure 1\Pass 1\01-Nov-2019-13-36-10HMS2LFS.gld'};
for k = 1:length(merfiles)
    fprintf('\n%s ', merfiles{k});
    [t1, t2] = GLDAlign(merfiles{k}, eogfiles{k});
    GLDViewAligned(t1, t2);
end
