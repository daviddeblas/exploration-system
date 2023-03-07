import { mount } from "@vue/test-utils";
import Home from "../Home.vue";
import { describe, it, expect, beforeEach, vi } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { ROBOT_STATUS } from "@/common/constants";
import { SocketTestHelper } from "@/helper/socket-test-helper";

describe("Home", () => {
  let wrapper: any;
  let socketTestHelper: any;

  beforeEach(() => {
    socketTestHelper = new SocketTestHelper();
    wrapper = mount(Home, {
      global: { provide: { [socketProvider as symbol]: socketTestHelper } },
    });
  });

  it("should display rover and drone status as offline by default", () => {
    const roverStatus = wrapper.find(".robot_state").text();
    expect(roverStatus).toBe("Le rover est hors ligne");

    const droneStatus = wrapper.findAll(".robot_state").at(1)!.text();
    expect(droneStatus).toBe("Le drone est hors ligne");
  });

  it("should update rover and drone status when receiving rover_state event", async () => {
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.offline);
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.offline);

    socketTestHelper.peerSideEmit("rover_state", true);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.in_mission);

    socketTestHelper.peerSideEmit("rover_state", false);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.pending);

    socketTestHelper.peerSideEmit("rover_state", undefined);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.offline);

    socketTestHelper.peerSideEmit("drone_state", true);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.in_mission);

    socketTestHelper.peerSideEmit("drone_state", false);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.pending);

    socketTestHelper.peerSideEmit("drone_state", undefined);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.offline);
  });

  it("should emit start event when start button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const startButton = wrapper.find(".btn:nth-child(1)");
    startButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("start", {
      data: "Démarrer mission",
    });
  });

  it("should emit identify event when identify button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const identifyButton = wrapper.find(".btn:nth-child(2)");
    identifyButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("identify", {
      data: "beep",
    });
  });

  it("should emit stop event when finish button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const stopButton = wrapper.find(".btn:nth-child(3)");
    stopButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("finish", {
      data: "Finir mission",
    });
  });
});
