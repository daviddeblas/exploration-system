import { mount } from "@vue/test-utils";
import Logs from "../Logs.vue";
import { describe, it, expect, beforeEach, vi } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { SocketTestHelper } from "@/helper/socket-test-helper";
import { SERVER_URL } from "@/common/constants";

describe("Logs", () => {
  let wrapper: any;
  let socketTestHelper: any;
  beforeEach(() => {
    socketTestHelper = new SocketTestHelper();
    wrapper = mount(Logs, {
      global: { provide: { [socketProvider as symbol]: socketTestHelper } },
    });
  });

  it("loads logs and missions on mounted", async () => {
    await wrapper.vm.$nextTick();
    expect(wrapper.vm.missions).toBeTruthy();
    expect(wrapper.vm.logs).toBeTruthy();
  });

  it("adds log when onLogger is called with a new log", async () => {
    const newLog = {
      id: 1,
      mission_id: 1,
      time: "2022-03-24T16:41:16.000Z",
      robot: "rover",
      category: "data",
      data: "Test log",
    };
    wrapper.vm.onLogger(JSON.stringify(newLog));
    expect(wrapper.vm.logs.length).toEqual(1);
    expect(wrapper.vm.logs[0]).toEqual(newLog);
  });

  it("loads next logs when onNext is called", async () => {
    wrapper.vm.logs = [
      {
        id: 2,
        mission_id: 1,
        time: "2022-03-24T16:41:16.000Z",
        robot: "rover",
        category: "data",
        data: "Test log",
      },
    ];
    wrapper.vm.start_id = 2;
    await wrapper.vm.onNext();
    expect(wrapper.vm.logs.length).toBeGreaterThan(0);
  });

  it("should display logs for selected mission", async () => {
    // Set the missions and logs data in the component
    wrapper.setData({
      missions: [
        { id: 1, start: "2023-04-15T10:00:00" },
        { id: 2, start: "2023-04-15T11:00:00" },
      ],
      logs: [
        {
          id: 1,
          mission_id: 1,
          time: "2023-04-15T10:05:00",
          robot: "R1",
          category: "Event1",
          data: "Data1",
        },
        {
          id: 2,
          mission_id: 1,
          time: "2023-04-15T10:10:00",
          robot: "R2",
          category: "Event2",
          data: "Data2",
        },
        {
          id: 3,
          mission_id: 2,
          time: "2023-04-15T11:05:00",
          robot: "R3",
          category: "Event3",
          data: "Data3",
        },
        {
          id: 4,
          mission_id: 2,
          time: "2023-04-15T11:10:00",
          robot: "R4",
          category: "Event4",
          data: "Data4",
        },
      ],
    });

    // On selectionne la mission 1
    wrapper.find("select").setValue(1);
    await wrapper.vm.$nextTick();
    const logs = wrapper.findAll("tr");

    expect(logs.length).toBe(5); // 4 logs + 1 header
    expect(logs[1].text()).toContain("R1");
    expect(logs[1].text()).toContain("Event1");
    expect(logs[1].text()).toContain("Data1");
    expect(logs[2].text()).toContain("R2");
    expect(logs[2].text()).toContain("Event2");
    expect(logs[2].text()).toContain("Data2");
  });
});
