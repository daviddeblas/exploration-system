import { mount } from "@vue/test-utils";
import Home from "../Home.vue";
import { describe, it, expect, beforeEach } from "vitest";
import { socketProvider } from "@/plugins/socket";
import { ROBOT_STATUS } from "@/common/constants";
import { ref } from "vue";

describe("Home", () => {
  let mockSocket: any;
  let rover: any;
  let drone: any;

  beforeEach(() => {
    const rover = ref(ROBOT_STATUS.offline);
    const drone = ref(ROBOT_STATUS.offline);
    mockSocket = {
      on: () => {
        return;
      },
      emit: () => {
        return;
      },
    };
  });

  it("should display rover status as offline by default", () => {
    // const onStub = vi.fn();
    // const emitStub = vi.fn();
    // const stubSocket = { on: onStub, emit: emitStub };
    const wrapper = mount(Home, { mocks: { $socket: mockSocket } });
    const roverStatus = wrapper.find(".robot_state").text();
    expect(roverStatus).toBe("Le rover est hors ligne");
  });
});
