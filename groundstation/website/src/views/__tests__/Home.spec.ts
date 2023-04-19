import { mount } from "@vue/test-utils";
import Home from "../Home.vue";
import { describe, it, expect, beforeEach, vi } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { ROBOT_STATUS } from "@/common/constants";
import { SocketTestHelper } from "@/helper/socket-test-helper";

URL.createObjectURL = vi.fn(() => "mocked-image-url");

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
    const roverStatus = wrapper.find(".rover_status").text();
    expect(roverStatus).toBe("Limo Status: hors ligne");
    const droneStatus = wrapper.find(".drone_status").text();
    expect(droneStatus).toBe("Cognifly Status: hors ligne");
  });

  it("should update rover and drone status when receiving rover_state event", async () => {
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.offline);
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.offline);
    socketTestHelper.peerSideEmit("rover_state", "True");
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.in_mission);
    socketTestHelper.peerSideEmit("rover_state", "False");
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.pending);
    socketTestHelper.peerSideEmit("rover_state", undefined);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.rover).toBe(ROBOT_STATUS.offline);
    socketTestHelper.peerSideEmit("drone_state", "True");
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.in_mission);
    socketTestHelper.peerSideEmit("drone_state", "False");
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.pending);
    socketTestHelper.peerSideEmit("drone_state", undefined);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.drone).toBe(ROBOT_STATUS.offline);
  });

  it("should emit start event when start button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const startButton = wrapper.find("#buttons button:nth-child(1)");
    startButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("start", {
      data: "Démarrer mission",
    });
  });

  it("should emit identify event when identify button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const identifyButton = wrapper.find("#buttons button:nth-child(2)");
    identifyButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("identify", {
      data: "beep",
    });
  });

  it("should emit stop event when finish button is clicked", () => {
    vi.spyOn(socketTestHelper, "emit");
    const stopButton = wrapper.find("#buttons button:nth-child(3)");
    stopButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("finish", {
      data: "Finir mission",
    });
  });

  it("should emit p2p event when P2P button is clicked", () => {
    socketTestHelper.emit = vi.fn();
    const p2pButton = wrapper.find("#buttons button:nth-child(4)");
    p2pButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("p2p", {
      data: "activer le mode P2P",
    });
  });

  it("should emit return_home event when return home button is clicked", () => {
    socketTestHelper.emit = vi.fn();
    const returnHomeButton = wrapper.find("#buttons button:nth-child(5)");
    returnHomeButton.trigger("click");
    expect(socketTestHelper.emit).toHaveBeenCalledWith("return_home", {
      data: "Retour à la base",
    });
  });

  it("should display updated battery states in the UI", async () => {
    socketTestHelper.peerSideEmit("rover_state", "True");
    socketTestHelper.peerSideEmit("drone_state", "True");
    await wrapper.vm.$nextTick();
    const droneBatteryState = wrapper.find(".drone_battery");
    const roverBatteryState = wrapper.find(".rover_battery");
    expect(droneBatteryState.text()).toBe("Cognifly Battery: 100%");
    expect(roverBatteryState.text()).toBe("Limo Battery: 100%");
    const droneBattery = 80;
    const roverBattery = 70;
    socketTestHelper.peerSideEmit("drone_battery", droneBattery);
    await wrapper.vm.$nextTick();
    expect(droneBatteryState.text()).toContain(
      `Cognifly Battery: ${droneBattery}%`
    );
    socketTestHelper.peerSideEmit("rover_battery", roverBattery);
    await wrapper.vm.$nextTick();
    expect(roverBatteryState.text()).toContain(
      `Limo Battery: ${roverBattery}%`
    );
  });

  it("should update mapImageUrl when receiving map_update event", async () => {
    const mapData = new ArrayBuffer(8);
    socketTestHelper.peerSideEmit("map_update", mapData);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.mapImageUrl).not.toBe("");
  });

  it("should update mapImageCogniflyUrl when receiving map_cognifly_update event", async () => {
    const mapData = new ArrayBuffer(8);
    socketTestHelper.peerSideEmit("map_cognifly_update", mapData);
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.mapImageCogniflyUrl).not.toBe("");
  });

  it("should display the updated map and CogniflyMap images in the UI", async () => {
    const mapData = new ArrayBuffer(8);
    const cogniflyData = new ArrayBuffer(8);

    socketTestHelper.peerSideEmit("map_update", mapData);
    await wrapper.vm.$nextTick();
    const mapImage = wrapper.find('.image-container img[alt="Map"]');
    expect(mapImage.exists()).toBe(true);

    socketTestHelper.peerSideEmit("map_cognifly_update", cogniflyData);
    await wrapper.vm.$nextTick();
    const cogniflyImage = wrapper.find(
      '.image-container img[alt="CogniflyMap"]'
    );
    expect(cogniflyImage.exists()).toBe(true);
  });
});
