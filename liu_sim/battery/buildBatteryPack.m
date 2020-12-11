%% Constructs a batterypack from cell data
function batpack = buildBatteryPack(batcell)
    batpack.parallel_cells  = batcell.NoCells.parallel;
    batpack.series_cells    = batcell.NoCells.series;
    batpack.SOC             = batcell.SOC;  % Initial state of charge
    batpack.SOC_max         = batcell.SOC_max;
    batpack.SOC_min         = batcell.SOC_min;
    
    batpack.cell_count  = batpack.parallel_cells .* batpack.series_cells;
    
    batpack.Imax        = batcell.Imax .* batpack.parallel_cells;
    batpack.Imin        = batcell.Imin .* batpack.parallel_cells;
    
    batpack.R       = batpack.series_cells ./ (batpack.parallel_cells ./ batcell.R);
    batpack.m       = batcell.m .* batpack.parallel_cells .* batpack.series_cells;
    batpack.Vnom    = batcell.Vnom .* batpack.series_cells;
    batpack.Voc     = batcell.Voc .* batpack.series_cells;
    batpack.socix   = batcell.socix;
    batpack.Q       = batcell.Q .* batpack.parallel_cells;
    
    batpack = polyfitBatterySOC(batpack);
end