package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.objectweb.asm.ClassReader;
import org.objectweb.asm.ClassVisitor;
import org.objectweb.asm.ClassWriter;
import org.objectweb.asm.MethodVisitor;
import org.objectweb.asm.Opcodes;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

/**
 * Changes the robot code's time calls to SimClock calls, while the classes load. Nothing in the
 * robot code is edited on disk. When no sim is running SimClock passes every call to the real
 * clock, so other tests are not affected.
 *
 * Only calls are changed. A class keeps the same fields, methods and stack use, so it behaves the
 * same except for what time it sees.
 *
 * The Android build compiles tests against android.jar, which has no java.lang.instrument, so that
 * part is reached through reflection.
 */
public final class ClockRewriter {

    private static final String CLOCK = "org/firstinspires/ftc/teamcode/kalipsorobotics/sim/SimClock";
    private static final String OP_MODE_UTILITIES = "org/firstinspires/ftc/teamcode/kalipsorobotics/utilities/OpModeUtilities";

    private static boolean installed;

    /** Safe to call more than once. Call before the robot classes are used. */
    public static synchronized void install() throws ReflectiveOperationException {
        if (installed) {
            return;
        }
        Object instrumentation = Class.forName("net.bytebuddy.agent.ByteBuddyAgent").getMethod("install").invoke(null);
        Class<?> instrumentationType = Class.forName("java.lang.instrument.Instrumentation");
        Class<?> transformerType = Class.forName("java.lang.instrument.ClassFileTransformer");

        // The JVM calls transform(...) with the class name and bytes as the last arguments (the
        // number of arguments differs between Java versions), so they are picked from the end.
        Object transformer = Proxy.newProxyInstance(ClockRewriter.class.getClassLoader(), new Class<?>[]{transformerType},
                (proxy, method, args) -> {
                    if (method.getName().equals("transform") && args != null && args.length >= 5) {
                        return rewrite((String) args[args.length - 4], (byte[]) args[args.length - 1]);
                    }
                    return null;
                });
        instrumentationType.getMethod("addTransformer", transformerType, boolean.class).invoke(instrumentation, transformer, true);

        // Classes that an earlier test in this JVM already loaded are rewritten too.
        Method isModifiable = instrumentationType.getMethod("isModifiableClass", Class.class);
        Method retransform = instrumentationType.getMethod("retransformClasses", Class[].class);
        Class<?>[] loadedClasses = (Class<?>[]) instrumentationType.getMethod("getAllLoadedClasses").invoke(instrumentation);
        for (Class<?> loaded : loadedClasses) {
            if ((Boolean) isModifiable.invoke(instrumentation, loaded) && shouldRewrite(loaded.getName().replace('.', '/'))) {
                try {
                    retransform.invoke(instrumentation, (Object) new Class<?>[]{loaded});
                } catch (Throwable ignored) {
                    // a class we cannot change keeps the real clock; the timing check in the run will show it
                }
            }
        }
        installed = true;
    }

    private static boolean shouldRewrite(String className) {
        if (className.startsWith("org/firstinspires/ftc/teamcode/kalipsorobotics/sim/")) {
            return false; // the sim itself must use the real clock
        }
        return className.startsWith("org/firstinspires/ftc/teamcode/")
                || className.equals("com/qualcomm/robotcore/util/ElapsedTime")
                || className.equals("com/qualcomm/robotcore/eventloop/opmode/OpMode")
                || className.equals("com/qualcomm/robotcore/eventloop/opmode/LinearOpMode");
    }

    /** Returns the rewritten class bytes, or null when the class has no time calls to change. */
    private static byte[] rewrite(String className, byte[] classBytes) {
        if (className == null || !shouldRewrite(className)) {
            return null;
        }
        try {
            ClassReader reader = new ClassReader(classBytes);
            ClassWriter writer = new ClassWriter(reader, 0);
            boolean[] changed = {false};
            reader.accept(new ClassVisitor(Opcodes.ASM9, writer) {
                @Override
                public MethodVisitor visitMethod(int access, String name, String descriptor, String signature, String[] exceptions) {
                    MethodVisitor next = super.visitMethod(access, name, descriptor, signature, exceptions);
                    return new MethodVisitor(Opcodes.ASM9, next) {
                        @Override
                        public void visitMethodInsn(int opcode, String owner, String methodName, String methodDescriptor, boolean isInterface) {
                            if (replaceCall(this, opcode, owner, methodName, methodDescriptor)) {
                                changed[0] = true;
                            } else {
                                super.visitMethodInsn(opcode, owner, methodName, methodDescriptor, isInterface);
                            }
                        }
                    };
                }
            }, 0);
            return changed[0] ? writer.toByteArray() : null;
        } catch (Throwable t) {
            System.err.println("ClockRewriter: could not rewrite " + className + ": " + t);
            return null;
        }
    }

    /** Writes the SimClock call instead of the real one. Returns false if this call is not a time call. */
    private static boolean replaceCall(MethodVisitor out, int opcode, String owner, String name, String descriptor) {
        if (opcode == Opcodes.INVOKESTATIC) {
            String target = null;
            if (owner.equals("java/lang/System") && name.equals("nanoTime") && descriptor.equals("()J")) {
                target = "nanoTime";
            } else if (owner.equals("java/lang/System") && name.equals("currentTimeMillis") && descriptor.equals("()J")) {
                target = "currentTimeMillis";
            } else if (owner.equals("java/lang/Thread") && name.equals("sleep") && descriptor.equals("(J)V")) {
                target = "sleep";
            } else if (owner.equals("java/lang/Thread") && name.equals("yield") && descriptor.equals("()V")) {
                target = "yield";
            } else if (owner.equals("android/os/SystemClock")) {
                if (name.equals("elapsedRealtime") || name.equals("elapsedRealtimeNanos") || name.equals("uptimeMillis")) {
                    target = name;
                } else if (name.equals("sleep")) {
                    target = "systemClockSleep";
                }
            }
            if (target != null) {
                out.visitMethodInsn(Opcodes.INVOKESTATIC, CLOCK, target, descriptor, false);
                return true;
            }
        }
        if (opcode == Opcodes.INVOKESTATIC && owner.equals("java/util/concurrent/Executors")
                && name.equals("newSingleThreadExecutor") && descriptor.equals("()Ljava/util/concurrent/ExecutorService;")) {
            // threads the code starts (the odometry thread) join the scheduler when they are CREATED, in a fixed order
            out.visitMethodInsn(Opcodes.INVOKESTATIC, CLOCK, "newSingleThreadExecutor", descriptor, false);
            return true;
        }
        if (opcode == Opcodes.INVOKEVIRTUAL && name.equals("opModeIsActive") && descriptor.equals("()Z")) {
            // a loop that polls opModeIsActive() before START would spin without ever letting time move
            out.visitMethodInsn(Opcodes.INVOKESTATIC, CLOCK, "opModeIsActive", "(Ljava/lang/Object;)Z", false);
            return true;
        }
        if (opcode == Opcodes.INVOKEVIRTUAL && owner.equals(OP_MODE_UTILITIES)
                && name.equals("clearBulkCache") && descriptor.equals("()V")) {
            out.visitMethodInsn(Opcodes.INVOKESTATIC, CLOCK, "loopBoundary", "(Ljava/lang/Object;)V", false);
            return true;
        }
        return false;
    }
}
