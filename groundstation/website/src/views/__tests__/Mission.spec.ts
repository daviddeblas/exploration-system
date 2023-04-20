import { mount } from "@vue/test-utils";
import Missions from "../Missions.vue";
import { describe, it, expect, beforeEach, afterEach } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { SocketTestHelper } from "@/helper/socket-test-helper";

describe("Missions", () => {
  let wrapper: any;
  let socketTestHelper: any;

  beforeEach(() => {
    socketTestHelper = new SocketTestHelper();
    wrapper = mount(Missions, {
      global: { provide: { [socketProvider as symbol]: socketTestHelper } },
    });
  });

  afterEach(() => {
    socketTestHelper.disconnect();
  });

  it("renders missions list correctly", async () => {
    const mission = {
      id: "1",
      start: "2023-04-18T00:00:00",
      end: "2023-04-18T01:30:45",
      has_rover: true,
      has_drone: false,
      distance_rover: 0,
      distance_drone: 0,
      is_sim: false,
    };

    wrapper.vm.missions = [mission];
    await wrapper.vm.$nextTick();

    const missionElement = wrapper.get("table");
    expect(missionElement.text()).toContain("Début");
    expect(missionElement.text()).toContain("Durée");
    expect(missionElement.text()).toContain("Robots");
    expect(missionElement.text()).toContain("Rover");
    expect(missionElement.text()).toContain("Distance parcourue (rover)");
    expect(missionElement.text()).not.toContain("Distance parcourue (drone)");
    expect(missionElement.text()).toContain("Simulation");
  });

  it("updates missions list when a mission update is received", () => {
    const existingMission = {
      id: "1",
      start: "2023-04-18T00:00:00",
      end: "2023-04-18T01:30:45",
      has_rover: true,
      has_drone: false,
      distance_rover: 0,
      distance_drone: 0,
      is_sim: false,
    };

    const updatedMission = {
      id: "1",
      start: "2023-04-18T00:00:00",
      end: "2023-04-18T02:00:00",
      has_rover: true,
      has_drone: true,
      distance_rover: 10,
      distance_drone: 20,
      is_sim: false,
    };

    wrapper.vm.missions = [existingMission];
    wrapper.vm.onMissionUpdate(JSON.stringify(updatedMission));

    expect(wrapper.vm.missions[0]).toEqual(updatedMission);
    expect(wrapper.vm.missions.length).toBe(1);
  });

  it("calculates mission duration", () => {
    const mission = {
      id: "1",
      start: "2023-04-18T00:00:00",
      end: "2023-04-18T01:30:45",
      has_rover: true,
      has_drone: false,
      distance_rover: 0,
      distance_drone: 0,
      is_sim: false,
    };

    const duration = wrapper.vm.missionDuration(mission);
    expect(duration).toBe("90m45s");
  });

  it("rounds a number to one decimal place", () => {
    const value = 12.345;
    const roundedValue = wrapper.vm.round(value);
    expect(roundedValue).toBe(12.3);
  });
});
