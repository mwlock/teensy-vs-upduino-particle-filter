%  Load dat files

num_files = 80;
samples_per_file = 70000;

% preallocate arrays
t = zeros(samples_per_file * num_files,1);
x_1 = zeros(samples_per_file * num_files,1);
x_2 = zeros(samples_per_file * num_files,1);

% Iterate through files
for i = 1:num_files
    
    % Load data
    file = sprintf( 'SDS%05d.DAT', i ) ;
    array = readcell(file);
    array = cell2mat(array);

    % Append data
    t(samples_per_file*(i-1)+1:samples_per_file*i) = array(:,1);
    x_1(samples_per_file*(i-1)+1:samples_per_file*i) = array(:,2)*10;
    x_2(samples_per_file*(i-1)+1:samples_per_file*i) = array(:,3)*10;

    if i > 1
        t(samples_per_file*(i-1)+1:samples_per_file*i) = t(samples_per_file*(i-1)+1:samples_per_file*i) + t(samples_per_file*(i-1));
    end

    clear array;

    disp(i)

end

x_3 = x_1 - x_2;

% Print stats
fprintf('Average voltage drop : %f \n',mean(x_3))

% Save file
filename = 'experiment.mat';
save(filename)