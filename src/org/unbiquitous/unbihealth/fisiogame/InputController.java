package org.unbiquitous.unbihealth.fisiogame;

import com.fastdtw.dtw.FastDTW;
import com.fastdtw.dtw.TimeWarpInfo;
import com.fastdtw.timeseries.TimeSeries;
import com.fastdtw.timeseries.TimeSeriesBase;
import com.fastdtw.util.DistanceFunction;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.apache.commons.math3.complex.Quaternion;
import org.unbiquitous.unbihealth.imu.IMUDriver;
import org.unbiquitous.unbihealth.imu.Sample;
import org.unbiquitous.uos.core.UOS;
import org.unbiquitous.uos.core.UOSLogging;
import org.unbiquitous.uos.core.messageEngine.dataType.UpDevice;
import org.unbiquitous.uos.core.messageEngine.messages.Call;
import org.unbiquitous.uos.core.messageEngine.messages.Response;
import org.unbiquitous.uos.network.socket.TCPProperties;
import org.unbiquitous.uos.network.socket.radar.MulticastRadar;

import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.filechooser.FileFilter;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Predicate;
import java.util.logging.Level;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;
import java.util.regex.Pattern;

/**
 * Shows a very simple interface to connect to an IMU driver and make service calls and process input for Fisiogame.
 *
 * @author Luciano Santos
 */
public class InputController extends JFrame {
    private static UOSThread uosThread;

    static {
        UOSLogging.setLevel(Level.FINE);
    }

    public static void main(String args[]) throws Exception {
        UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        SwingUtilities.invokeLater(() -> {
            InputController controller = new InputController();
            controller.setDefaultCloseOperation(DISPOSE_ON_CLOSE);
            controller.setMinimumSize(new Dimension(600, 600));
            controller.setSize(new Dimension(1024, 768));
            controller.setLocationRelativeTo(null);
            controller.addWindowListener(new WindowAdapter() {
                @Override
                public void windowClosing(WindowEvent e) {
                    uosThread.stop();
                    System.exit(0);
                }
            });
            controller.setVisible(true);
        });
        (new Thread(uosThread = new UOSThread())).start();
    }

    private Map<String, TimeSeries> curvesMap = new HashMap<>();

    public InputController() {
        super("Fisiogame Input Controller");

        initialize();

        txtIP.setText(loadLastIP());
    }

    private static final Color DARK_GREEN = new Color(0, 128, 0, 255);
    private JTextField txtIP;
    private JButton btnTest;
    private JComboBox<String> cboSensorIds;
    private JLabel lblIPTestResult;
    private JTextArea txtLog;
    private JTextField txtFilePath;
    private JFileChooser fileChooser;
    private JButton btnBrowse;
    private JButton btnLoad;
    private JList<String> lstCurves;
    private DefaultListModel<String> lstModel;
    private JButton btnRemove;
    private JButton btnStartRecording;
    private JButton btnStopRecording;
    private JLabel lblRecordId;
    private JButton btnMatch;

    private void initialize() {
        setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        gbc.gridx = gbc.gridy = 0;
        gbc.weightx = gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;

        JPanel ipPane = new JPanel(new GridBagLayout());
        ipPane.setBorder(BorderFactory.createTitledBorder("Host"));
        GridBagConstraints ipGbc = new GridBagConstraints();
        ipGbc.gridx = ipGbc.gridy = 0;
        ipGbc.weightx = ipGbc.weighty = 1.0;
        ipGbc.fill = GridBagConstraints.BOTH;

        JLabel lblAddress = new JLabel("IMU Host Address:");
        ipGbc.weightx = 0.0;
        ipGbc.insets = new Insets(5, 5, 5, 0);
        ipPane.add(lblAddress, ipGbc);

        txtIP = new JTextField();
        txtIP.getDocument().addDocumentListener(new DocumentListener() {
            @Override
            public void insertUpdate(DocumentEvent e) {
                txtIPChanged();
            }

            @Override
            public void removeUpdate(DocumentEvent e) {
                txtIPChanged();
            }

            @Override
            public void changedUpdate(DocumentEvent e) {
                txtIPChanged();
            }
        });

        ipGbc.gridx++;
        ipGbc.weightx = 1.0;
        ipGbc.insets = new Insets(5, 5, 5, 0);
        ipPane.add(txtIP, ipGbc);

        btnTest = new JButton("Test");
        btnTest.setForeground(DARK_GREEN);
        btnTest.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                testIP();
            }
        });
        ipGbc.gridx++;
        ipGbc.weightx = 0.0;
        ipGbc.insets = new Insets(5, 5, 5, 5);
        ipPane.add(btnTest, ipGbc);

        JLabel lblSensorId = new JLabel("Sensor ID:");
        ipGbc.gridx = 0;
        ipGbc.gridy++;
        ipGbc.weightx = 0.0;
        ipGbc.insets = new Insets(5, 5, 5, 0);
        ipPane.add(lblSensorId, ipGbc);

        cboSensorIds = new JComboBox<>();
        cboSensorIds.addItemListener(new ItemListener() {
            @Override
            public void itemStateChanged(ItemEvent e) {
                btnStartRecording.setEnabled((!btnStopRecording.isEnabled()) && (cboSensorIds.getSelectedIndex() >= 0));
            }
        });
        ipGbc.gridx++;
        ipGbc.weightx = 1.0;
        ipPane.add(cboSensorIds, ipGbc);

        lblIPTestResult = new JLabel(" ");
        ipGbc.gridx++;
        ipGbc.weightx = 0.2;
        ipPane.add(lblIPTestResult, ipGbc);

        gbc.weighty = 0.0;
        gbc.insets = new Insets(5, 5, 0, 5);
        add(ipPane, gbc);

        JPanel curvesPane = new JPanel(new GridBagLayout());
        curvesPane.setBorder(BorderFactory.createTitledBorder("Curves"));
        GridBagConstraints curvesGbc = new GridBagConstraints();
        curvesGbc.gridx = curvesGbc.gridy = 0;
        curvesGbc.weightx = curvesGbc.weighty = 0.0;
        curvesGbc.fill = GridBagConstraints.BOTH;

        JLabel lblFilePath = new JLabel("Files:");
        curvesGbc.insets = new Insets(5, 5, 0, 0);
        curvesPane.add(lblFilePath, curvesGbc);

        txtFilePath = new JTextField();
        curvesGbc.gridx++;
        curvesGbc.weightx = 1.0;
        curvesGbc.insets = new Insets(5, 5, 0, 0);
        curvesPane.add(txtFilePath, curvesGbc);

        btnBrowse = new JButton("Browse");
        btnBrowse.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                btnBrowseActionPerformed();
            }
        });
        curvesGbc.gridx++;
        curvesGbc.weightx = 0.0;
        curvesGbc.insets = new Insets(5, 5, 0, 0);
        curvesPane.add(btnBrowse, curvesGbc);

        btnLoad = new JButton("Load");
        btnLoad.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                btnLoadActionPerformed();
            }
        });
        curvesGbc.gridx++;
        curvesGbc.weightx = 0.0;
        curvesGbc.insets = new Insets(5, 5, 0, 5);
        curvesPane.add(btnLoad, curvesGbc);

        lstModel = new DefaultListModel<>();
        lstCurves = new JList<>(lstModel);
        lstCurves.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        lstCurves.addListSelectionListener(new ListSelectionListener() {
            @Override
            public void valueChanged(ListSelectionEvent e) {
                boolean selected = lstCurves.getSelectedIndex() >= 0;
                btnRemove.setEnabled(selected);
                btnMatch.setEnabled(btnStopRecording.isEnabled() && selected);
            }
        });
        lstCurves.addKeyListener(new KeyAdapter() {
            @Override
            public void keyReleased(KeyEvent e) {
                if ((e.getKeyCode() == KeyEvent.VK_DELETE) && btnRemove.isEnabled())
                    btnRemoveActionPerformed();
            }
        });
        JScrollPane listPane = new JScrollPane(lstCurves);
        curvesGbc.gridx = 0;
        curvesGbc.gridy++;
        curvesGbc.gridwidth = 4;
        curvesGbc.weightx = curvesGbc.weighty = 1.0;
        curvesGbc.insets = new Insets(5, 5, 0, 5);
        curvesPane.add(listPane, curvesGbc);

        JPanel recordButtonsPane = new JPanel(new FlowLayout(FlowLayout.LEFT));
        btnStartRecording = new JButton("REC");
        btnStartRecording.setEnabled(false);
        btnStartRecording.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                startRecording();
            }
        });
        recordButtonsPane.add(btnStartRecording);
        btnStopRecording = new JButton("STOP");
        btnStopRecording.setEnabled(false);
        btnStopRecording.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                stopRecording();
            }
        });
        recordButtonsPane.add(btnStopRecording);
        btnMatch = new JButton("MATCH");
        btnMatch.setEnabled(false);
        btnMatch.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                match();
            }
        });
        recordButtonsPane.add(btnMatch);
        curvesGbc.gridx = 0;
        curvesGbc.gridy++;
        curvesGbc.gridwidth = 3;
        curvesGbc.weightx = 1.0;
        curvesGbc.weighty = 0.0;
        curvesGbc.insets = new Insets(5, 5, 5, 5);
        curvesPane.add(recordButtonsPane, curvesGbc);

        btnRemove = new JButton("Remove");
        btnRemove.setEnabled(false);
        btnRemove.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                btnRemoveActionPerformed();
            }
        });
        curvesGbc.gridx = 3;
        curvesGbc.gridwidth = 1;
        curvesGbc.weightx = 0.0;
        curvesGbc.insets = new Insets(5, 0, 5, 5);
        curvesPane.add(btnRemove, curvesGbc);

        lblRecordId = new JLabel();
        curvesGbc.gridx = 0;
        curvesGbc.gridy++;
        curvesGbc.gridwidth = 4;
        curvesGbc.weightx = 1.0;
        curvesGbc.weighty = 0.0;
        curvesGbc.insets = new Insets(5, 5, 5, 5);
        curvesPane.add(lblRecordId, curvesGbc);

        gbc.gridy++;
        gbc.weighty = 1.0;
        gbc.insets = new Insets(5, 5, 0, 5);
        add(curvesPane, gbc);

        txtLog = new JTextArea();
        txtLog.setEditable(false);
        JScrollPane logPane = new JScrollPane(txtLog);
        logPane.setBorder(BorderFactory.createTitledBorder("Error Log"));
        gbc.gridy++;
        gbc.weighty = 0.5;
        gbc.insets = new Insets(5, 5, 5, 5);
        add(logPane, gbc);
    }

    private void txtIPChanged() {
        final Pattern HOST_NAME = Pattern.compile(
                "^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$");
        final Color NORMAL = UIManager.getDefaults().getColor("TextField.foregroundColor");
        final Color ERROR = Color.ORANGE;

        boolean valid = HOST_NAME.matcher(txtIP.getText()).matches();
        txtIP.setForeground(valid ? NORMAL : ERROR);
        lblIPTestResult.setText(" ");
        clearSensorIds();
    }

    private void clearSensorIds() {
        cboSensorIds.removeAllItems();
        cboSensorIds.setSelectedIndex(-1);
    }

    private static final FileFilter JSON_FILTER = new FileFilter() {
        @Override
        public boolean accept(File f) {
            return f.isDirectory() || f.getName().toLowerCase().endsWith(".json");
        }

        @Override
        public String getDescription() {
            return "JSON files (*.json)";
        }
    };

    private void btnBrowseActionPerformed() {
        if (showFileDialog(DialogType.OPEN, JSON_FILTER, true) == JOptionPane.OK_OPTION) {
            StringBuffer paths = new StringBuffer();
            for (File f : fileChooser.getSelectedFiles()) {
                paths.append(f.getAbsolutePath());
                paths.append(File.pathSeparator);
            }
            txtFilePath.setText(paths.toString());
        }
    }

    private void btnLoadActionPerformed() {
        log(null);
        String[] paths = txtFilePath.getText().split(";");
        StringWriter sw = new StringWriter();
        PrintWriter out = new PrintWriter(sw);
        String path;
        for (String p : paths) {
            if ((path = p.trim()).isEmpty())
                continue;
            try {
                String json = new String(Files.readAllBytes(Paths.get(path)), Charset.forName("UTF-8"));
                curvesMap.put(path, processCurve(IMUDriver.extractRecordedData(json)));
            } catch (IOException e) {
                out.println("Failed to load file '" + path + "':");
                e.printStackTrace(out);
            }
        }
        String errors = sw.toString();
        if (!errors.isEmpty())
            log(errors);

        lstModel.clear();
        for (String p : curvesMap.keySet())
            lstModel.addElement(p);

        txtFilePath.setText("");
    }

    private void btnRemoveActionPerformed() {
        int index = lstCurves.getSelectedIndex();
        String path = lstModel.get(index);
        lstModel.remove(index);
        lstCurves.setSelectedIndex(-1);
        curvesMap.remove(path);
    }

    private void testIP() {
        log(null);
        btnTest.setEnabled(false);
        clearSensorIds();
        Call call = new Call(IMUDriver.DRIVER_NAME, IMUDriver.LIST_IDS_NAME);
        uosThread.callService(getDevice(), call, (resp, errMsg) -> {
            boolean sucess = false;
            if (errMsg != null)
                log(errMsg);
            else {
                try {
                    List<String> ids = IMUDriver.extractIdList(resp);
                    for (String id : ids)
                        cboSensorIds.addItem(id);
                    saveLastIP(txtIP.getText().trim());
                    sucess = true;
                } catch (Throwable t) {
                    log(toString(t));
                }
            }
            lblIPTestResult.setForeground(sucess ? DARK_GREEN : Color.red);
            lblIPTestResult.setText(sucess ? "success!" : "fail!");
            btnTest.setEnabled(true);
        });
    }

    private void startRecording() {
        log(null);
        btnStartRecording.setEnabled(false);
        Call call = new Call(IMUDriver.DRIVER_NAME, IMUDriver.TARE_NAME);
        uosThread.callService(getDevice(), call, (resp, errMsg) -> {
            if (errMsg != null)
                log(errMsg);
            else
                onTared();
        });
    }

    private void onTared() {
        Call call = new Call(IMUDriver.DRIVER_NAME, IMUDriver.START_RECORD_NAME);
        call.addParameter(IMUDriver.SENSOR_ID_PARAM_NAME, cboSensorIds.getSelectedItem());
        uosThread.callService(getDevice(), call, (resp, errMsg) -> {
            String recordId = null;
            if (errMsg != null)
                log(errMsg);
            else
                recordId = resp.getResponseString(IMUDriver.RECORD_ID_PARAM_NAME);

            boolean success = recordId != null;
            btnStopRecording.setEnabled(success);
            btnMatch.setEnabled(success && (lstCurves.getSelectedIndex() >= 0));
            btnStartRecording.setEnabled(!success);
            lblRecordId.setText(recordId);
        });
    }

    private void stopRecording() {
        retrieveCurve(null);
    }

    private void match() {
        retrieveCurve(lstModel.elementAt(lstCurves.getSelectedIndex()));
    }

    private void retrieveCurve(final String refCurveId) {
        log(null);
        btnStopRecording.setEnabled(false);
        btnMatch.setEnabled(false);
        Call call = new Call(IMUDriver.DRIVER_NAME, IMUDriver.STOP_RECORD_NAME);
        call.addParameter(IMUDriver.SENSOR_ID_PARAM_NAME, cboSensorIds.getSelectedItem());
        call.addParameter(IMUDriver.RECORD_ID_PARAM_NAME, lblRecordId.getText());
        uosThread.callService(getDevice(), call, (resp, errMsg) -> {
            btnStartRecording.setEnabled(true);
            lblRecordId.setText(null);

            if (errMsg != null)
                log(errMsg);
            else {
                try {
                    onCurveRetrieved(IMUDriver.extractRecordedData(resp), refCurveId);
                } catch (Throwable t) {
                    log(toString(t));
                }
            }
        });
    }

    private static final DistanceFunction WORST_DIST = new DistanceFunction() {
        @Override
        public double calcDistance(double[] v1, double[] v2) {
            if (v1.length != v2.length)
                throw new RuntimeException("vectors sizes don't match");
            double max = Double.MIN_VALUE;
            for (int i = 0; i < v1.length; ++i)
                max = Math.max(max, Math.abs(v1[i] - v2[i]));
            return max;
        }
    };

    private void onCurveRetrieved(List<Sample> samples, String refCurveId) throws IOException {
        TimeSeries ts = processCurve(samples);
        if (refCurveId != null) {
            TimeSeries base = curvesMap.get(refCurveId);
            TimeWarpInfo info = FastDTW.compare(base, ts, WORST_DIST);
            log(info.toString());
        } else if (showFileDialog(DialogType.SAVE, JSON_FILTER, false) == JOptionPane.OK_OPTION) {
            final ObjectMapper mapper = new ObjectMapper();
            mapper.writerWithDefaultPrettyPrinter().writeValue(fileChooser.getSelectedFile(), samples);
            saveCSV(fileChooser.getSelectedFile().getAbsolutePath() + ".csv", samples);
            saveCSV(fileChooser.getSelectedFile().getAbsolutePath() + "-trimmed.csv", ts);
        }
    }

    private static final double CURVE_THRESHOLD = 0.025;

    private TimeSeries processCurve(List<Sample> list) {
        Sample[] samples = new Sample[list.size()];
        list.toArray(samples);

        // Using the threshold, verifies the trim points for the array...
        final Predicate<double[]> lowerThanThreshold = (s) -> {
            return Arrays.stream(s).map(Math::abs).max().getAsDouble() <= CURVE_THRESHOLD;
        };
        int first = 0, last = samples.length - 1;
        Quaternion qf = samples[first].getQuaternion(), ql = samples[last].getQuaternion();
        while ((first < last) && lowerThanThreshold.test(relativeTo(samples[first].getQuaternion(), qf)))
            ++first;
        while ((last > first) && lowerThanThreshold.test(relativeTo(samples[last].getQuaternion(), ql)))
            --last;

        // Generates the series.
        TimeSeriesBase.Builder builder = TimeSeriesBase.builder();
        long baseTime = samples[first].getTimestamp();
        for (int k = first; k <= last; ++k)
            builder.add(samples[k].getTimestamp() - baseTime, relativeTo(samples[k].getQuaternion(), qf));
        return builder.build();
    }

    private static double[] relativeTo(Quaternion q, Quaternion base) {
        return new double[]{
                q.getQ0() - base.getQ0(),
                q.getQ1() - base.getQ1(),
                q.getQ2() - base.getQ2(),
                q.getQ3() - base.getQ3()
        };
    }

    private void saveCSV(String path, List<Sample> curve) {
        log(null);
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("t;s;x;y;z\n");
            Sample base = curve.get(0);
            for (Sample s : curve) {
                long t = s.getTimestamp() - base.getTimestamp();
                double[] d = relativeTo(s.getQuaternion(), base.getQuaternion());
                fw.write(String.format("%d;%2.12f;%2.12f;%2.12f;%2.12f\n", t, d[0], d[1], d[2], d[3]));
            }
        } catch (Throwable t) {
            log(toString(t));
        }
    }

    private void saveCSV(String path, TimeSeries curve) {
        log(null);
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("t;s;x;y;z\n");
            for (int i = 0; i < curve.size(); ++i) {
                long t = (long) curve.getTimeAtNthPoint(i);
                double[] d = curve.getMeasurementVector(i);
                fw.write(String.format("%d;%2.12f;%2.12f;%2.12f;%2.12f\n", t, d[0], d[1], d[2], d[3]));
            }
        } catch (Throwable t) {
            log(toString(t));
        }
    }

    private void log(String msg) {
        txtLog.setText(msg);
    }

    private UpDevice getDevice() {
        return new UpDevice("imu").addNetworkInterface(txtIP.getText().trim(), "Ethernet:TCP");
    }

    private enum DialogType {OPEN, SAVE}

    private int showFileDialog(DialogType type, FileFilter filter, boolean multSelect) {
        fileChooser = new JFileChooser(loadDialogPath());
        fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
        fileChooser.setMultiSelectionEnabled(multSelect);
        fileChooser.setFileFilter(filter);
        int result = JFileChooser.CANCEL_OPTION;
        switch (type) {
            case OPEN:
                result = fileChooser.showOpenDialog(this);
                break;

            case SAVE:
                result = fileChooser.showSaveDialog(this);
                break;
        }
        if (result == JFileChooser.APPROVE_OPTION)
            saveDialogPath(fileChooser.getSelectedFile().getParentFile());

        return result;
    }

    private String loadLastIP() {
        return Preferences.userRoot().get("HostIP", "");
    }

    private void saveLastIP(String ip) {
        try {
            Preferences userPrefs = Preferences.userRoot();
            userPrefs.put("HostIP", ip);
            userPrefs.sync();
        } catch (BackingStoreException e) {
            e.printStackTrace();
        }
    }

    private File loadDialogPath() {
        return new File(Preferences.userRoot().get("DialogPath", "."));
    }

    private void saveDialogPath(File path) {
        try {
            Preferences userPrefs = Preferences.userRoot();
            userPrefs.put("DialogPath", path.getAbsolutePath());
            userPrefs.sync();
        } catch (BackingStoreException e) {
            e.printStackTrace();
        }
    }

    private static final Font materialFont;

    private static String toString(Throwable t) {
        StringWriter sw = new StringWriter();
        t.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }

    static {
        try {
            materialFont = Font.createFont(
                    Font.TRUETYPE_FONT, InputController.class.getResourceAsStream("/MaterialIcons-Regular.ttf"));
        } catch (Exception e) {
            throw new RuntimeException("Couldn't load internal font.", e);
        }
    }

    private static class UOSThread implements Runnable {
        private volatile UOS uos = null;
        private volatile boolean started = false;

        @Override
        public void run() {
            if (started)
                throw new RuntimeException("UOS instance alread running.");

            uos = new UOS();
            TCPProperties props = new MulticastRadar.Properties();
            props.setPort(8300);
            props.setPassivePortRange(8301, 8310);
            props.put("ubiquitos.multicast.beaconFrequencyInSeconds", 10);
            uos.start(props);
            started = true;
        }

        public void stop() {
            if (!started)
                return;

            try {
                uos.stop();
            } catch (Throwable e) {
                e.printStackTrace();
            }
            started = false;
        }

        public void callService(UpDevice device, Call call, BiConsumer<Response, String> callback) {
            if (!started)
                callback.accept(null, "UOS not started yet.");
            new Thread(() -> {
                try {
                    Response r = uos.getGateway().callService(device, call);
                    callback.accept(r, r.getError());
                } catch (Exception e) {
                    callback.accept(null, InputController.toString(e));
                }
            }).start();
        }
    }
}
