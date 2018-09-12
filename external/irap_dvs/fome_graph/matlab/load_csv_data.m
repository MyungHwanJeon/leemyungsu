disp('opening event csv file..');
dvs_file = '/home/jhlee/data/demo_thermal/dvs_events.csv';
dvs_file_id = fopen(dvs_file);
dvs_headers = textscan(dvs_file_id, '%s', 24062, 'Delimiter', ',');
dvs_data = textscan(dvs_file_id, '%s', 3859,  'Delimiter', '');
disp('finished scannning event csv file.');
fclose(dvs_file_id);

disp('processing event array into stream of sampledvs...');
sampledvs = {};
dvscounter = 1;
for i=1:1:length(dvs_data{1,1})
    eventarray = strsplit(dvs_data{1,1}{i},',');
    for j=1:1:(length(eventarray)/5 - 7)
        event.x = str2int(eventarray{5*j+3});
        event.y = str2int(eventarray{5*j+4});
        event.t = str2double(eventarray{5*j+5}) + 1e-9 .* str2double(eventarray{5*j+6});
        event.p = str2bool(eventarray{5*j+7});
        sampledvs{dvscounter} = event;
        dvscounter = dvscounter + 1;
    end
end
disp('finished processing event stream.');