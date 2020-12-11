function v = OPTI_PSD()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 132);
  end
  v = vInitialized;
end
