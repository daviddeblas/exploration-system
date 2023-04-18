export interface Log {
  id: number;
  mission_id: number;
  time: string;
  robot: string;
  category: string;
  data: string;
}

export interface Mission {
  id: number;
  start: string;
  end: string;
  is_sim: boolean;
  has_rover: boolean;
  has_drone: boolean;
  distance_rover: number;
  distance_drone: number;
}
