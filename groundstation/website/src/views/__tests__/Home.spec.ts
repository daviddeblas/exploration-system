import { mount } from "@vue/test-utils";
import Home from "../Home.vue";
import { describe, it, expect, beforeEach, vi } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { ROBOT_STATUS } from "@/common/constants";
import { ref } from "vue";

describe("Home", () => {
  let mockSocket: any;
  let rover: any;
  let drone: any;

  beforeEach(() => {
    mockSocket = { on: vi.fn(), emit: vi.fn() };
    rover = ref(ROBOT_STATUS.offline);
    drone = ref(ROBOT_STATUS.offline);
  });

  it("should display rover and drone status as offline by default", () => {
    const wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: mockSocket } },
    });

    const roverStatus = wrapper.find(".robot_state").text();
    expect(roverStatus).toBe("Le rover est hors ligne");

    const droneStatus = wrapper.findAll(".robot_state").at(1)!.text();
    expect(droneStatus).toBe("Le drone est hors ligne");
  });

  it("should update rover and drone status when receiving rover_state event", async () => {
    const wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: mockSocket } },
    });

    await wrapper.vm.$nextTick();
    expect(rover.value).toBe(ROBOT_STATUS.offline);
    expect(drone.value).toBe(ROBOT_STATUS.offline);

    // mockSocket.on.mockImplementation((eventName: string, callback: any) => {
    //   if (eventName === "rover_state") {
    //     callback(true);
    //     expect(rover.value).toBe(ROBOT_STATUS.in_mission);
    //     callback(false);
    //     expect(rover.value).toBe(ROBOT_STATUS.pending);
    //     callback(undefined);
    //     expect(rover.value).toBe(ROBOT_STATUS.offline);
    //   }
    //   if (eventName === "drone_state") {
    //     callback(true);
    //     expect(drone.value).toBe(ROBOT_STATUS.in_mission);
    //     callback(false);
    //     expect(drone.value).toBe(ROBOT_STATUS.pending);
    //     callback(undefined);
    //     expect(drone.value).toBe(ROBOT_STATUS.offline);
    //   }
    // });
  });

  it("should emit start event when start button is clicked", () => {
    const wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: mockSocket } },
    });
    const startButton = wrapper.find(".btn:nth-child(1)");
    startButton.trigger("click");
    expect(mockSocket.emit).toHaveBeenCalledWith("start", {
      data: "Démarrer mission",
    });
  });

  it("should emit identify event when identify button is clicked", () => {
    const wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: mockSocket } },
    });
    const identifyButton = wrapper.find(".btn:nth-child(2)");
    identifyButton.trigger("click");
    expect(mockSocket.emit).toHaveBeenCalledWith("identify", { data: "beep" });
  });

  it("should emit stop event when finish button is clicked", () => {
    const wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: mockSocket } },
    });
    const stopButton = wrapper.find(".btn:nth-child(3)");
    stopButton.trigger("click");
    expect(mockSocket.emit).toHaveBeenCalledWith("finish", {
      data: "Finir mission",
    });
  });
});
