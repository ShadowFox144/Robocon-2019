void init_buff() {
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < N; j++)
    {
      Vsp_readings[i][j] = 0;
    }
  }
}

void cal_Vsp_average()
{
  for (int i = 0; i < 3; i++)
  {
    Vsp_summation[i] = Vsp_summation[i] - Vsp_readings[i][Vsp_readIndex[i]];

    Vsp_readings[i][Vsp_readIndex[i]] = Vsp[i];

    Vsp_summation[i] = Vsp_summation[i] + Vsp_readings[i][Vsp_readIndex[i]];

    Vsp_readIndex[i] = Vsp_readIndex[i] + 1;

    if (Vsp_readIndex[i] >= Vsp_N) {
      Vsp_readIndex[i] = 0;
    }
    Vsp_average[i] = Vsp_summation[i] / Vsp_N;
  }
}
